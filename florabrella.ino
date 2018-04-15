#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>


/** Configuration **/

/* Flora v2 neopixel data pin */
const int pixel_pin = 8;

/* Neopixel strip layout */
const int strip_pin = 6;
const int geometry[] = {18, 18, 19, 19, 19, 19, 19, 19};
const int nPixels = 150;

/* Pushbutton Switch */
const int switchPin = 10;

/* Brightness */
const uint8_t brightness = 255;

/* Framerate (in milliseconds):
 * advisory as a maximum framerate; i.e. frames will never render FASTER than
 * this rate. Animations may use this for converting anticipated durations into
 * some number of arbitrary animation frames. */
const uint16_t framerate = 20;


/** Global State **/

const int nStrips = sizeof(geometry) / sizeof(geometry[0]);
int stripAddrs[nStrips];

float lum[nPixels]; // Luminance map for pixel strip
int buttonState;    // variable to hold the button state
int lightMode;      // how many times the button has been pressed

uint8_t pColorRGB[3]; // Current color RGB
uint32_t pcolor;      // Current color

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
void setColor(uint8_t r, uint8_t g, uint8_t b)
{
    pColorRGB[0] = r;
    pColorRGB[1] = g;
    pColorRGB[2] = b;
    pcolor = strip.Color(r, g, b);
    pixel.setPixelColor(0, pcolor);
    pixel.show();
}

void setColor(uint32_t color)
{
    uint8_t b = (color & 0xff);
    uint8_t g = (color >> 8) & 0xff;
    uint8_t r = (color >> 16) & 0xff;
    setColor(r, g, b);
}

/** Invoke the sensor to read a color value */
uint32_t readColor() {
  int i;

  // this sequence flashes the first pixel three times as a countdown to the color reading.
  for (i = 0; i < 3; i++) {
    //white, but dimmer-- 255 for all three values makes it blinding!
    setColor(188, 188, 188);
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

/**
 * Animation FN:
 * Rotate the color on the on-board pixel, setting the color each time.
 */
void color_chooser(void)
{
  static uint8_t frame = 0;
  uint16_t i;
  uint32_t c;

  c = wheel(frame % 256);
  frame = (frame + 1) % 256;
  setColor(c);
}

void lum_adjust(int16_t addr, float amt)
{
  lum[addr] += amt;
  if (lum[addr] > 1.00) {
    lum[addr] = 1.00;
  } else if (lum[addr] < 0.0) {
    lum[addr] = 0.0;
  }
}

/* Given a strip and a local address, get the global address */
int16_t loc2global(int8_t strip, int8_t loc)
{
  return stripAddrs[strip] + loc;
}

/* Given the luminance map, apply the static, color color
 * to all the pixels in the strip. */
void renderFromLuminance(void)
{
    int16_t i;

    for (i = 0; i < strip.numPixels(); i++) {
      uint8_t r, g, b;

      r = lum[i] * pColorRGB[0];
      g = lum[i] * pColorRGB[1];
      b = lum[i] * pColorRGB[2];

      r = gammatable[r];
      g = gammatable[g];
      b = gammatable[b];

      strip.setPixelColor(i, r, g, b);
    }
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

/**
 * sparkles: creates a number of sparkles that appear and fade out.
 * @nSparkles: maximum number of concurrent sparkes
 * @pSparkle:    Odds that a new sparkle will be created.
 *               If p = 40 and frame rate is 20ms,
 *               40 * 20ms --> 800ms;
 *               A sparkle can be expected every 800ms.
 * @sparkleLife: maximum life of a sparkle, in frames.
 *               If the frame rate is 20, a sparkle life of 100 is 2000ms,
 *               2 seconds.
 */
bool mode_sparkles(void) {
    struct sparkle {
        int8_t strand;
        int8_t strandPos;
        int16_t pos;
        float brightness;
        bool visible;
        unsigned long tick;
    };
    /* Animation parameters */
    static const int nSparkles = 4;
    static const int pSparkle = 40;
    static const int sparkleLife = 100;
    /* state */
    static struct sparkle spark[nSparkles];
    static unsigned long tick;
    int i, j, k;

    /* Start with no color, and no brightness */
    solidColor(0);
    for (i = 0; i < strip.numPixels(); i++) {
      lum[i] = 0.0;
    }

    /* Iterate over the sparkles */
    for (i = 0; i < nSparkles; i++) {
        if (spark[i].visible) {
            int frame = tick - spark[i].tick;
            float decay = ((exp(2 * ((float)frame / (float)sparkleLife)) - 1) / exp(2));
            float ell = spark[i].brightness - decay;

            /* Sparkle is extinguished: */
            if (ell <= 0.0) {
                ell = 0.0;
                spark[i].visible = false;
                continue;
            }

            /* For all pixels on this sparkle's strand, compute brightness; with k=0 as the epicenter: */
            for (j = 0; j < geometry[spark[i].strand]; j++) {
              float fade_lum;
              k = abs(j - spark[i].strandPos);
              fade_lum = ell / exp(k);
              lum_adjust(loc2global(spark[i].strand, j), fade_lum);
            }
        } else if (random(pSparkle) == 0) {
            /* Create a new sparkle: */
            spark[i].visible = true;
            spark[i].tick = tick;
            /* Brightness is somewhere from 0.75 to 1.00 */
            spark[i].brightness = 1.00 - (((float)random(100) / 100.0) * 0.25);
            spark[i].strand = random(nStrips);
            spark[i].strandPos = random(geometry[spark[i].strand]);
            spark[i].pos = stripAddrs[spark[i].strand] + spark[i].strandPos;
        }
    }

    renderFromLuminance();
    tick += 1;
    return false;
}

bool mode_paparazzi(void)
{
    /* time in ms for a light to fade to 1% */
    static uint16_t life = 1000;
    static uint16_t lifeFrames = life / framerate;
    static float decay = pow(100.0, 1.0 / (float)lifeFrames);
    static float decayCoefficient = 1.0 / decay;

    int16_t i;
    int16_t addr = random(strip.numPixels());

    /* Buzzap! */
    lum[addr] = 1.00;

    /* Decay remaining sparks */
    for (i = 0; i < strip.numPixels(); i++) {
        if (lum[i]) {
            lum[i] = lum[i] * decayCoefficient;
            if (lum[i] <= 0.01) {
                lum[i] = 0.00;
            }
        }
    }

    renderFromLuminance();
    return false;
}

bool mode_detect(void) {
  uint32_t c = readColor();
  setColor(c);
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
  MODE_SPARKLES,
  MODE_PAPARAZZI,
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
  [MODE_SPARKLES]      = mode_sparkles,
  [MODE_PAPARAZZI]     = mode_paparazzi,
  [MODE_DETECT]        = mode_detect
};

void setup() {
  int i, j;

  // Set up serial communication at 9600bps
  Serial.begin(9600);

  // Set up the built-in pixel for the Flora
  pixel = Adafruit_NeoPixel(1, pixel_pin, NEO_GRB + NEO_KHZ800);
  pixel.begin();
  setColor(0, 255, 0);

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
  if (brightness != 255) {
    strip.setBrightness(brightness);
  }
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
  unsigned long dx;

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

  t2 = millis();
  dx = t2 - t1;
  t1 = t2;
  if (dx < framerate) {
      delay(framerate - dx);
  }
}
