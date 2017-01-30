
// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include "FastLED.h"
#define COLOR_ORDER GRB
#define MAX_BRIGHTNESS 255
//Tell it how many leds are in the strip. AndyMark's 2.5 meter strip has 150 leds
#define NUM_LEDS 265
// This is an array of leds. One item for each led in your strip
CRGB leds[NUM_LEDS];
//CSK 3/17/2014 I moved this to a pin that doesn't conflict with Ethernet functions in case you want to controlLEDs via Ethernet
#define DATA_PIN 6 //White wire from the http://www.andymark.com/product-p/am-2917.htm powerconnector
//This function is used to setup things like pins, Serial ports etc.
//Here we specify which chipset our LEDs run off of by our choice of config function
 CRGB Conductors;

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

void setup() {
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  //lcd.print("Conductors 4580");
  lcd.setBacklight(WHITE);
 FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
 FastLED.clear();
 FastLED.show();
 delay(250);
 //clear() turns all LEDs off
 FastLED.clear();
 FastLED.setBrightness(MAX_BRIGHTNESS);
 fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 125, 125, 125) );
 FastLED.show();
 Conductors.r = 148;
 Conductors.g = 0;
 Conductors.b = 211;
}
void rainbow(uint8_t wait)
{
 uint16_t hue;
 FastLED.clear();
 for(hue=10; hue<255*3; hue++)
 {
 fill_rainbow( &(leds[0]), NUM_LEDS /*led count*/, hue /*starting hue*/);
 FastLED.show();
 delay(wait);
 }
 return;
}
//Cylon - LED sweeps back and forth, with the color, delay and number of cycles of your choice
void cylon(CRGB color, uint16_t wait, uint8_t number_of_cycles)
{
 FastLED.setBrightness(MAX_BRIGHTNESS);
 for (uint8_t times = 0; times<=number_of_cycles; times++)
 {
 // Make it look like one LED is moving in one direction
 for(int led_number = 0; led_number < NUM_LEDS; led_number++)
 {
 //Apply the color that was passed into the function
 leds[led_number] = color;
 //Actually turn on the LED we just set
 FastLED.show();
 // Turn it back off
 leds[led_number] = CRGB::Black;
 // Pause before "going" to next LED
 delay(wait);
 }
 // Now "move" the LED the other direction
 for(int led_number = NUM_LEDS-1; led_number >= 0; led_number--)
 {
 //Apply the color that was passed into the function
 leds[led_number] = color;
 //Actually turn on the LED we just set
 FastLED.show();
 // Turn it back off
 leds[led_number] = CRGB::Black;
 // Pause before "going" to next LED
 delay(wait);
 }
 }
 return;
}
uint8_t i=0;
void loop() {
  uint8_t buttons = lcd.readButtons();
      while(true) {
        String inputSerial = Serial.readString();
        if (inputSerial == "rainbow") {
          //cylon(Conductors,0,1);
          rainbow(1);
        }
  }
}

//These are the functions we have defined to do chase patterns. They are actually called inside the loop() above
//They are meant to demonstrate things such as setting LED colors, controlling brightness
void color_chase(uint32_t color, uint8_t wait)
{
 FastLED.clear();
 //The brightness ranges from 0-255
 //Sets brightness for all LEDS at once
 FastLED.setBrightness(MAX_BRIGHTNESS);
 // Move a block of LEDs

 for(int led_number = 0; led_number < NUM_LEDS - 5; led_number++)
 {
 // Turn our current led ON, then show the leds
 leds[led_number] = color;
 //CSK 4/22/2016 Make it multiple dots on
 leds[led_number + 1] = color;
 leds[led_number + 2] = color;
 leds[led_number + 3] = color;
 leds[led_number + 4] = color;
 leds[led_number + 5] = color;
 // Show the leds (only one of which is has a color set, from above
 // Show turns actually turns on the LEDs
 FastLED.show();
 // Wait a little bit
 delay(wait);
 // Turn our current led back to black for the next loop around
 //CSK 4/22/2016 Turn the dots off
 leds[led_number] = CRGB::Black;
 }
 return;
}
//Move an "empty" dot down the strip
void missing_dot_chase(uint32_t color, uint8_t wait)
{
 //Step thru some brightness levels from max to 10. led_brightness/=2 is a cryptic shorthand way of sayingled_brightness = led_brightness/2
 // for (int led_brightness = MAX_BRIGHTNESS; led_brightness > 10; led_brightness/=2)
 {
 //FastLED.setBrightness(led_brightness);
 //CSK 4/22/2016 Turn brightness down to save batteries since almost all leds are on
 FastLED.setBrightness(25);
 // Start by turning all pixels on:
 //for(int led_number = 0; led_number < NUM_LEDS; led_number++) leds[led_number] = color;
 //https://github.com/FastLED/FastLED/wiki/Controlling-leds
 fill_solid(leds, NUM_LEDS, color);
 // Then display one pixel at a time:
 for(int led_number = 0; led_number < NUM_LEDS - 5; led_number++)
 {
 leds[led_number] = CRGB::Black; // Set new pixel 'off'
 //CSK 4/22/2016
 leds[led_number + 1] = CRGB::Black; // Set new pixel 'off'
 leds[led_number + 2] = CRGB::Black; // Set new pixel 'off'
 leds[led_number + 3] = CRGB::Black; // Set new pixel 'off'
 leds[led_number + 4] = CRGB::Black; // Set new pixel 'off'
 leds[led_number + 5] = CRGB::Black; // Set new pixel 'off'
 if( led_number > 0 && led_number < NUM_LEDS)
 {
 leds[led_number-1] = color; // Set previous pixel 'on'
 }
 FastLED.show();
 delay(wait);
 }
 }
 return;
}

