/******************************************************************************
  Example_07_MicBias.ino
  This example demonstrates control of the mic bias feature of WM8960 Codec.

  Electret Mics are powered with a mic bias voltage applied to their signal 
  line - usually with a 2.2K resistor in series. This Codec can provide a clean 
  mic bias.
  
  This example turns on the mic bias, set's it to each available output voltage,
  and then turns it off to demonstrate disable.

  Measure the voltage with a multimeter to verfy you are getting the correct
  voltages you desire on mic bias.

  You can later use this mic bias voltage to power an electret mic in a more 
  advanced example (example 14).

  Development platform used:
  SparkFun ESP32 IoT RedBoard v10

  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

  Pete Lewis @ SparkFun Electronics
  October 14th, 2022
  https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library
  
  This code was created using some code by Mike Grusin at SparkFun Electronics
  Included with the LilyPad MP3 example code found here:
  Revision history: version 1.0 2012/07/24 MDG Initial release
  https://github.com/sparkfun/LilyPad_MP3_Player

  Do you like this library? Help support SparkFun. Buy a board!

    SparkFun Audio Codec Breakout - WM8960 (QWIIC)
    https://www.sparkfun.com/products/21250
	
	All functions return 1 if the read/write was successful, and 0
	if there was a communications failure. You can ignore the return value
	if you just don't care anymore.

	For information on the data sent to and received from the CODEC,
	refer to the WM8960 datasheet at:
	https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions 
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/

#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> 
// Click here to get the library: http://librarymanager/All#SparkFun_WM8960
WM8960 codec;

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 7 - MicBias Control");

  Wire.begin();

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); // Freeze
  }
  Serial.println("Device is connected properly.");

  // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  codec.enableMicBias();

  // WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or 
  // WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
  codec.setMicBiasVoltage(WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD); 
  Serial.println("Mic Bias enabled (0.9*AVDD)");
  delay(3000);

  // WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or 
  // WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
  codec.setMicBiasVoltage(WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD); 
  Serial.println("Mic Bias enabled (0.65*AVDD)");
  delay(3000);

  codec.disableMicBias();
  Serial.println("Mic Bias disabled");
  
  Serial.println("Example Complete. Hit reset to begin again.");
}

void loop()
{
  // Nothing to see here
}