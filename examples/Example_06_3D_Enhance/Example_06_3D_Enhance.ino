/******************************************************************************
  Example _06_3D_Enhance.ino
  Demonstrates 3D Enhance feature of WM8960 Codec.

  Toggles on/off 3D Enhance every 5 seconds, so you can hear the difference.

  Note, it also sets the 3D enhance Depth setting to 15 (max).
  You can change this to your desired depth from 0-15.

  Note, this is very similar to the Loopback example, but also demos the 3D Enhance feature.

  Sets up analog audio input (on INPUT1s), ADC/DAC Loopback, sets volume control, and Headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT1" inputs, 
  they are labeled "RIN1" and "LIN1" on the board.

  This example will pass your audio source through the mixers and gain stages of the codec 
  into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
  Then send the output of the DAC to the headphone outs.

  You can now control the volume of the codecs built in headphone amp using this fuction:

  codec.setHeadphoneVolume(120); Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB

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


  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_WM8960
WM8960 codec;

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 6 - 3D Enhance");

  Wire.begin();

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.println("Device is connected properly.");

  // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  // setup signal flow to the ADC

  codec.enable_LMIC();
  codec.enable_RMIC();
  
  // connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
  codec.connect_LMN1();
  codec.connect_RMN1();

  // disable mutes on PGA inputs (aka INTPUT1)
  codec.disable_LINMUTE();
  codec.disable_RINMUTE();

  // set input boosts to get inputs 1 to the boost mixers
  codec.set_LMICBOOST(0); // 0 = 0dB
  codec.set_RMICBOOST(0); // 0 = 0dB

  codec.connect_LMIC2B();
  codec.connect_RMIC2B();

  // enable boost mixers
  codec.enable_AINL();
  codec.enable_AINR();

  // disconnect LB2LO (booster to output mixer (analog bypass)
  // for this example, we are going to pass audio throught the ADC and DAC
  codec.disableLB2LO();
  codec.disableRB2RO();

  // connect from DAC outputs to output mixer
  codec.enableLD2LO();
  codec.enableRD2RO();

  // set gainstage between booster mixer and output mixer
  // for this loopback example, we are going to keep these as low as they go
  codec.setLB2LOVOL(0); // 0 = -21dB
  codec.setRB2ROVOL(0); // 0 = -21dB

  // enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d freq at 705.6kHz
  codec.enablePLL(); // needed for class-d amp clock
  codec.set_PLLPRESCALE(PLLPRESCALE_DIV_2);
  codec.set_SMD(PLL_MODE_FRACTIONAL);
  codec.set_CLKSEL(CLKSEL_PLL);
  codec.set_SYSCLKDIV(SYSCLK_DIV_BY_2);
  codec.set_BCLKDIV(4);
  codec.set_DCLKDIV(DCLKDIV_16);
  codec.set_PLLN(7);
  codec.set_PLLK(0x86, 0xC2, 0x26); // PLLK=86C226h	
  //codec.set_ADCDIV(0); // default is 000 (what we need for 44.1KHz), so no need to write this.
  //codec.set_DACDIV(0); // default is 000 (what we need for 44.1KHz), so no need to write this.

  codec.enableMasterMode(); 
  codec.set_ALRCGPIO(); // note, should not be changed while ADC is enabled.

  // enable ADCs and DACs
  codec.enableAdcLeft();
  codec.enableAdcRight();
  codec.enableDacLeft();
  codec.enableDacRight();
  codec.disableDacMute();

  codec.enableLoopBack(); // Loopback sends ADC data directly into DAC
  codec.disableDacMute(); // default is "soft mute" on, so we must disable mute to make channels active

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // provides VMID as buffer for headphone ground

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolume(120);
  Serial.println("3D Enhance Depth set to 15 (max)");
  codec.set3dDepth(15);

  Serial.println("Listen to left/right INPUT1 on Headphone outputs with toggling 3D enhance!");
}

void loop()
{
  codec.enable3d();
  Serial.println("3D Enhance enabled");
  delay(5000);
  
  codec.disable3d();
  Serial.println("3D Enhance disabled");
  delay(5000);
}