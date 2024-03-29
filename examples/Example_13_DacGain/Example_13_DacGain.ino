/******************************************************************************
  Example_13_DacGain.ino
  Demonstrates how to control the volume using the codec's DAC digital volume 
  control.

  Attach a potentiomenter to GND/A0/3V3 to actively adjust the setting.

  This example sets up the codec for analog audio input (on INPUT1s), ADC/DAC 
  Loopback, sets hp volume, and Headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT1" inputs, 
  they are labeled "RIN1" and "LIN1" on the board.

  This example will pass your audio source through the mixers and gain stages 
  of the codec into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
  Then send the output of the DAC to the headphone outs.

  We will use the gain stage at the DAC to control the volume of the signal. 
  This is capable of more precision, with 255 available settings.

  ** DAC digital volume
  ** Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
  ** -97.50 (or lower) = MUTE
  ** -97.00 = -97.00dB (MIN)
  ** ... 0.5dB steps ...
  ** 30.00 = +30.00dB  (MAX)

  You can also control the volume of the codecs built in headphone amp using 
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
  ESP32 -------- POTENTIOMTER (aka blue little trimpot)
  **********************
  GND ---------- "right-side pin"          
  A0 ----------- center pin            *aka center tap connection
  3V3 ---------- "left-side pin"             

  **********************
  CODEC ------- AUDIO IN
  **********************
  GND --------- TRS INPUT SLEEVE        *ground for line level input
  LINPUT1 ----- TRS INPUT TIP           *left audio
  RINPUT1 ----- TRS INPUT RING1         *right audio

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          
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

// Used to store incoming potentiometer settings to set ADC digital volume 
// setting
long userInputA0 = 0; 

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 13 - DAC Digital Volume Control");

  Wire.begin();

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); // Freeze
  }
  Serial.println("Device is connected properly.");

  codec_setup();
}

void loop()
{
  // Take a bunch of readings and average them, to smooth out the value
  for (int i = 0 ; i < 250 ; i ++) 
  {
    userInputA0 += analogRead(A0);
    delay(1);
  }

  // After taking a bunch of samples, divide down to the average single reading
  userInputA0 /= 250; 

  // Map it from 0-4096, to a dB value that is acceptable in the DAC digital 
  // volume control (-97.50 [MUTE] to +30dB)
  float dacVolumeDB = mapFloats((float)userInputA0, 0.00, 4096.00, 30.00, -97.50);

  Serial.print("dacVolumeDB: ");
  Serial.println(dacVolumeDB);
  
  codec.setDacLeftDigitalVolumeDB(dacVolumeDB); // -97.50 to +30.00dB
  codec.setDacRightDigitalVolumeDB(dacVolumeDB); // -97.50 to +30.00dB

  delay(50);
}

void codec_setup()
{
      // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  // Setup signal flow to the ADC

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

  // Disconnect LB2LO (booster to output mixer (analog bypass)
  // For this example, we are going to pass audio throught the ADC and DAC
  codec.disableLB2LO();
  codec.disableRB2RO();

  // Connect from DAC outputs to output mixer
  codec.enableLD2LO();
  codec.enableRD2RO();

  // Set gainstage between booster mixer and output mixer
  // For this loopback example, we are going to keep these as low as they go
  codec.setLB2LOVOL(WM8960_OUTPUT_MIXER_GAIN_NEG_21DB); 
  codec.setRB2ROVOL(WM8960_OUTPUT_MIXER_GAIN_NEG_21DB);

  // Enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d 
  // freq at 705.6kHz
  codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h	
  //codec.setADCDIV(0); // Default is 000 (what we need for 44.1KHz)
  //codec.setDACDIV(0); // Default is 000 (what we need for 44.1KHz)

  codec.enableMasterMode(); 
  codec.setALRCGPIO(); // Note, should not be changed while ADC is enabled.

  // Enable ADCs and DACs
  codec.enableAdcLeft();
  codec.enableAdcRight();
  codec.enableDacLeft();
  codec.enableDacRight();
  codec.disableDacMute();

  codec.enableLoopBack(); // Loopback sends ADC data directly into DAC

  // Default is "soft mute" on, so we must disable mute to make channels active
  codec.disableDacMute(); 

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  Serial.println("Headphopne Amp Volume set to +0dB");
  codec.setHeadphoneVolumeDB(0.00);

  Serial.println("Codec setup complete. Listen to left/right INPUT1 on Headphone outputs.");
}

// mapFloat
// This function is the same as the Arduino map() function, but it can accept
// and return float values. We need this to handle dB values which are floats
//
// value: the number to map.
//
// fromLow: the lower bound of the value’s current range.
//
// fromHigh: the upper bound of the value’s current range.
//
// toLow: the lower bound of the value’s target range.
//
// toHigh: the upper bound of the value’s target range.

float mapFloats(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}