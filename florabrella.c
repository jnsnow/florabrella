#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN 8

/* Layout of pixels */
const int strip_pin = 6;
int geometry[] = {18, 18, 19, 19, 19, 19, 19, 19};
const int nStrips = sizeof(geometry) / sizeof(geometry[0]);
int stripAddrs[nStrips];

int switchPin = 10; // switch is connected to pin 10
int buttonState;    // variable to hold the button state
int lightMode = 0;  // how many times the button has been pressed

uint32_t pcolor;    // Currently selected color

Adafruit_NeoPixel strip;
Adafruit_NeoPixel pixel;
Adafruit_TCS34725 tcs;
byte gammatable[256];

/** Helper Functions **/

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void solidColor(uint32_t c)
{
  uint16_t i;
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t wheel(byte wheelPos)
{
  if (wheelPos < 85) {
    return strip.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
  } else if (wheelPos < 170) {
    wheelPos -= 85;
    return strip.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  } else {
    wheelPos -= 170;
    return strip.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
}

/** Set the chosen color to the one specified */
void setColor(uint32_t color)
{
  pcolor = color;
  pixel.setPixelColor(0, color);
  pixel.show();
}

/** Invoke the sensor to read a color value */
uint32_t readColor() {
  int i;

  // this sequence flashes the first pixel three times as a countdown to the color reading.
  for (i = 0; i < 3; i++) {
    //white, but dimmer-- 255 for all three values makes it blinding!
    setColor(pixel.Color(188, 188, 188));
    delay(1000);
    setColor(0);
    delay(500);
  }

  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false); // turn on LED
  delay(60); // takes 50ms to read
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true); // turn off LED

  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);
  // Figure out some basic hex code for visualization
  uint32_t sum = red;
  sum += green;
  sum += blue;
  sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
  Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" "); Serial.println((int)b );

  return strip.Color(gammatable[(int)r], gammatable[(int)g], gammatable[(int)b]);
}


/** Light Mode Functions **/

typedef bool (*lightFn)(void);

bool mode_solid(void)
{
  solidColor(pcolor);
  return false;
}

bool mode_rainbow(void)
{
  static uint8_t frame = 0;
  uint16_t i;

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, wheel((i + frame) % 256));
  }

  frame = (frame + 1) % 256;
  return false;
}

bool mode_rain(void)
{
  struct raindrop {
    uint8_t strand;
    int16_t pos;
  };

  int i, s;
  const int count = 20;
  static struct raindrop drop[count];
  static bool initialized;

  // Set each rain drop at the starting gate.
  // Signify by a position of -1
  if (!initialized) {
    for (i = 0; i < count; i++) {
      drop[i].pos = -1;
    }
    initialized = true;
  }

  // Start with all LEDs off
  solidColor(0);

  // Loop for each raindrop
  for (i = 0; i < count; i++) {
    // Visible raindrops
    if (drop[i].pos >= 0) {
      /* FIXME: Catch drops at the end of their strand, not at the end of the strip */
      strip.setPixelColor(drop[i].pos, pcolor);
      drop[i].pos += 1;
      if (drop[i].pos > strip.numPixels()) {
        drop[i].pos = -1;
      }
    } else {
      // Non-visible raindrops:
      if (random(40) == 0) {
        drop[i].strand = random(nStrips);
        drop[i].pos = stripAddrs[drop[i].strand];
      }
    }
  }

  return false;
}

// Rainbow Cycle Program - Equally distributed
bool mode_rainbowCycle(void) {
  static uint16_t frame = 0;
  uint16_t i, j;

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, wheel(((i * 256 / strip.numPixels()) + frame) % 256));
  }

  /* fixme: is this a full cycle for this mode? */
  frame = (frame + 1) % (256 * 5);
  return false;
}

bool mode_blink(void) {
  static unsigned long t1;
  unsigned long t2;
  static int blink_ms = 250;
  static bool state;

  t2 = millis();
  if ((t2 - t1) >= blink_ms) {
    state = !state;
    t1 = t2;
  }

  if (state) {
    solidColor(pcolor);
  } else {
    solidColor(0);
  }

  return false;
}

bool mode_rotate(void)
{
  static unsigned long t1;
  unsigned long t2;
  static int rotate_ms = 250;
  static uint8_t s;
  int i;

  t2 = millis();
  if ((t2 - t1) >= rotate_ms) {
    s = (s + 1) % nStrips;
    t1 = t2;
  }

  solidColor(0);
  for (i = 0; i < geometry[s]; i++) {
    strip.setPixelColor(stripAddrs[s] + i, pcolor);
  }
  return false;
}

bool mode_void_rotate(void)
{
  static unsigned long t1;
  unsigned long t2;
  static int rotate_ms = 250;
  static uint8_t s;
  int i;

  t2 = millis();
  if ((t2 - t1) >= rotate_ms) {
    s = (s + 1) % nStrips;
    t1 = t2;
  }

  solidColor(pcolor);
  for (i = 0; i < geometry[s]; i++) {
    strip.setPixelColor(stripAddrs[s] + i, 0);
  }

  return false;
}

bool mode_detect(void) {
  pcolor = readColor();
  setColor(pcolor);
  colorWipe(pcolor, 20);

  return true;
}

enum lightMode {
  MODE__BEGIN        = 0,
  MODE_SOLID         = MODE__BEGIN,
  MODE_RAINBOW,
  MODE_RAIN,
  MODE_RAINBOW_CYCLE,
  MODE_BLINK,
  MODE_ROTATE,
  MODE_VOID_ROTATE,
  MODE_DETECT,
  /* Insert new modes here */
  MODE__MAX
};

lightFn modeDispatch[MODE__MAX] = {
  [MODE_SOLID]         = mode_solid,
  [MODE_RAINBOW]       = mode_rainbow,
  [MODE_RAIN]          = mode_rain,
  [MODE_RAINBOW_CYCLE] = mode_rainbowCycle,
  [MODE_BLINK]         = mode_blink,
  [MODE_ROTATE]        = mode_rotate,
  [MODE_VOID_ROTATE]   = mode_void_rotate,
  [MODE_DETECT]        = mode_detect
};

void setup() {
  int i, j;

  // Set up serial communication at 9600bps
  Serial.begin(9600);

  // Set up the built-in pixel for the Flora
  pixel = Adafruit_NeoPixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
  pixel.begin();
  setColor(pixel.Color(0, 255, 0));

  // Set up the button
  pinMode(switchPin, INPUT_PULLUP); // Set the switch pin as input
  buttonState = digitalRead(switchPin); // read the initial state

  // Set up the pixel strips
  for (i = j = 0; i < nStrips; i++) {
    stripAddrs[i] = j;
    j += geometry[i];
  }
  pinMode(strip_pin, OUTPUT);
  strip = Adafruit_NeoPixel(j, strip_pin, NEO_GRB + NEO_KHZ800);
  strip.setBrightness(80); //adjust brightness here
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Set up the color sensor
  tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    gammatable[i] = x;
    //Serial.println(gammatable[i]);
  }

  // Get color from sensor, set color, and show it:
  mode_detect();
}

void loop() {
  static unsigned long t1;
  unsigned long t2;
  int val;

  val = digitalRead(switchPin);
  if (val != buttonState && val == LOW) {
    lightMode = (lightMode + 1) % MODE__MAX;
  }
  buttonState = val;

  if (modeDispatch[lightMode]()) {
    lightMode = (lightMode + 1) % MODE__MAX;
  }

  strip.show();
  delay(20);
}
