
/*
 * This code included color reading from GY-33 Color sensor using
 * UART communication and send the readings to Bluetooth device and
 * OLED Display
 * 
 * Base dev board : ESP32
 * OLED Display : 128x32 - I2C
 * 
 * GY-33 Connections
 * 
 * VCC----VCC
 * GND----GND
 * GY33_TX---16,  
 * GY33_RX---17
 * 
 * written by: @isuruthiwa
 * 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT; 

/*
  GY-33
  GY33----ESP32
  VCC----VCC
  GND----GND
  1:GY33_TX---16,
  2:GY33_RX---17

  send A5 01 A6 to GY-33
*/

#define RXD2 16
#define TXD2 17
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
byte rgb[3] = {0};

void setup() {
  // put your setup code here, to run once:
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(10);
  Serial2.write(0XA5);
  Serial2.write(0x6A);    //Led level to 8
  Serial2.write(0x0F);
  delay(10);
  Serial2.write(0XA5);
  Serial2.write(0X81);    //Initialization, continuous output mode
  Serial2.write(0X26);    //Initialization, continuous output mode

  Serial.println("Started Reading...");
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);


}

void loop() {
  unsigned char i = 0, sum = 0;
  while (Serial2.available()) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if (counter == 0 && Re_buf[0] != 0x5A) return; // Check the frame header
    counter++;
    if (counter == 8)             //Data received
    {
      counter = 0;               //Re-assign the value to prepare for the reception of the next frame of data
      sign = 1;
    }
  }
  if (sign)
  {
    sign = 0;
    for (i = 0; i < 7; i++)
      sum += Re_buf[i];
    if (sum == Re_buf[i] )     //Check the frame head, frame end
    {
      rgb[0] = Re_buf[4];
      rgb[1] = Re_buf[5];
      rgb[2] = Re_buf[6];
      Serial.print("r:");
      Serial.print( rgb[0]);
      Serial.print(",g:");
      Serial.print( rgb[1]);
      Serial.print(",b:");
      Serial.println( rgb[2]);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Red : ");
      display.println(rgb[0]);
      display.print("Green : ");
      display.println(rgb[1]);
      display.print("Blue : ");
      display.print(rgb[2]);
      display.display();
      char stringBuffer80[80];
      sprintf (stringBuffer80, "R:%d, G:%d, B:%d \n\0", rgb[0], rgb[1], rgb[2]);

      int i = 0;
      while (stringBuffer80[i] != 0)
        SerialBT.write((uint8_t)stringBuffer80[i++]);

      delay(100);
    }
  }

  if (SerialBT.available()) {
    int c = SerialBT.parseInt();
    if (c >= 0 && c <= 10) {
      int number = c;
      Serial.print("BL message: ");
      Serial.println( number );

      delay(10);
      int16_t command = 0x60;
      command = command + number;
      Serial2.write(0XA5);
      Serial2.write(command);    //Led level to 8
      int16_t sum = 0xA5 + command;
      Serial2.write(sum);
    }
  }

}
