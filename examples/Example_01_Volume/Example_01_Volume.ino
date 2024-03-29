/******************************************************************************
  Example_01_Volume.ino
  Demonstrates analog audio input, volume control, and headphone output on the 
  WM8960 Codec.

  Audio should be connected to both the left and right "INPUT3" inputs, 
  they are labeled "RIN3" and "LIN3" on the board.

  This example will pass your audio source through the mixers and gain stages 
  of the codec using all of the analog bypass paths.

  It will output the sound on the headphone outputs. 
  It is setup to do a capless headphone setup, so connect your headphones ground 
  to "OUT3"vand this provides a buffered VMID.

  You can now control the volume of the codecs built in headphone buffers using 
  this function:

  codec.setHeadphoneVolumeDB(6.00); Valid inputs are -74.00 (MUTE) up to +6.00, 
  (1.00dB steps).

  Development platform used:
  SparkFun ESP32 IoT RedBoard v10

  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

  **********************
  CODEC ------- AUDIO IN
  **********************
  GND --------- TRS INPUT SLEEVE        *ground for line level input
  LINPUT3 ----- TRS INPUT TIP           *left audio
  RINPUT3 ----- TRS INPUT RING1         *right audio

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
  HPL ---------- TRS OUTPUT TIP             *left HP output
  HPR ---------- TRS OUTPUT RING1           *right HP output

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
  Serial.println("Example 1 - Volume");

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
  
  // Setup signal flow through the analog audio bypass connections

  // Enable left output mixer
  codec.enableLOMIX(); 

  // Enable bypass connection from Left INPUT3 to Left output mixer, note, the 
  // default gain on this input (LI2LOVOL) is -15dB
  codec.enableLI2LO(); 

  // Sets volume control between "left input" to "left output mixer"
  codec.setLI2LOVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); 
  
  codec.enableROMIX(); // Now for the right channel of INPUT3
  codec.enableRI2RO();
  codec.setRI2ROVOL(WM8960_OUTPUT_MIXER_GAIN_0DB);
  
  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  Serial.println("Volume set to +6.00dB (max)");
  codec.setHeadphoneVolumeDB(6.00);
  delay(5000);

  Serial.println("Volume set to +0.00dB");
  codec.setHeadphoneVolumeDB(0.00);
  delay(5000);

  Serial.println("Volume set to -12.00dB");
  codec.setHeadphoneVolumeDB(-12.00);
  delay(5000);  

  Serial.println("Volume set to -74.00dB, aka MUTE");
  codec.setHeadphoneVolumeDB(-74.00);
  delay(5000);

  Serial.println("Example complete. Hit Reset to try again.");
}

void loop()
{
  // Nothing to see here.
}
