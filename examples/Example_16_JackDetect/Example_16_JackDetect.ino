/******************************************************************************
  Example_16_JackDetect.ino
  Demonstrates Jack Detection Feature of the Codec.

  This example sets up analog audio input (on INPUT1s), and then uses jack detection
  to set the output to either the speakers or the headphones.

  Note, you will need a TRS connector that has a "shunt switch" on the ground sleeve.
  Something like the SparkFun Audio Jack - 1/4" Stereo (right angle) found here:
  https://www.sparkfun.com/products/11141

  When headphones are inserted into your TRS barrel jack input, this releases the 
  SHUNT pin on the TRS breakout. We will setup the codec to detect when headphones
  are inserted watching the LEFT INPUT 3 pin.

  Audio should be connected to both the left and right "INPUT1" inputs, 
  they are labeled "RIN1" and "LIN1" on the board.

  This example will pass your audio source through the mixers and gain stages of 
  the codec using all of the analog bypass paths.

  It will output the sound on the Speakers or the headphone outputs. 
  It is setup to do a capless headphone setup, so connect your headphones ground 
  to "OUT3" and this provides a buffered VMID.

  You can now control the volume of the codecs built in headphone buffers using 
  this function:

  codec.setHeadphoneVolumeDB(6.00); Valid inputs are -74.00 (MUTE) up to +6.00, 
  (1.00dB steps).

  Development platform used:
  SparkFun ESP32 IoT RedBoard v10

  HARDWARE CONNECTIONS

  **********************
  CODEC
  **********************
  LINPUT3 ----- 33K Resistor ----- AVDD (3.3V)

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
  LINPUT1 ----- TRS INPUT TIP           *left audio
  RINPUT1 ----- TRS INPUT RING1         *right audio
  LINPUT3 ----- TRS INPUT GND SHUNT     *GND shunt pin for headphone detect
                                        *Insertion of headphones will release this pin
                                        *from being connected to GND.

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
  HPL ---------- TRS OUTPUT TIP             *left HP output
  HPR ---------- TRS OUTPUT RING1           *right HP output

  Pete Lewis @ SparkFun Electronics
  October 30th, 2024
  https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

  Support for the jack detect feature was contributed by Harald Klien (github @haklein)
  October 2024. https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library/pull/9
  Thank you Harald!!
  
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
  Serial.println("Example 3 - INPUT1");

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

  codec.enableLMIC();
  codec.enableRMIC();
  
  // Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
  codec.connectLMN1();
  codec.connectRMN1();

  // Disable mutes on PGA inputs (aka INTPUT1)
  codec.disableLINMUTE();
  codec.disableRINMUTE();

  // Set input boosts to get inputs 1 to the boost mixers
  codec.setLMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);
  codec.setRMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);

  codec.connectLMIC2B();
  codec.connectRMIC2B();

  // Enable boost mixers
  codec.enableAINL();
  codec.enableAINR();

  // Connect LB2LO (booster to output mixer (analog bypass)
  codec.enableLB2LO();
  codec.enableRB2RO();

  // Set gainstage between booster mixer and output mixer
  codec.setLB2LOVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); 
  codec.setRB2ROVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); 

  // Enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();

  // Enable Speaker output
  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d 
  // freq at 705.6kHz
  codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h	

  codec.enableSpeakers();
  Serial.println("Speakers enabled.");
  codec.setSpeakerVolumeDB(0.00);
  Serial.println("Volume set to +0dB");

  // Enable Headphone output
  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground
  Serial.println("Headphones enabled.");
  codec.setHeadphoneVolumeDB(0.00);
  Serial.println("Volume set to +0dB");

  // Enable Jack Detect
  codec.enableHeadphoneJackDetect();
  Serial.println("Jack Detect enabled.");
  // Set input for Jack Detect input to LEFT INPUT 3
  codec.setHeadphoneJackDetectInput(WM8960_JACKDETECT_LINPUT3);
  Serial.println("Jack Detect input set to LEFT INPUT 3.");

  Serial.println("Example complete. Listen to INPUT1 on Speaker outputs.");
  Serial.println("Try plugging in headphones to switch to headphone output.");
  Serial.println("Unplug headphones to switch back to speakers.");
}

void loop()
{
  // Do nothing.
  // Plugging in headphones will automatically switch the output to the headphones.
  // Unplugging headphones will automatically switch the output to the speakers.
}
