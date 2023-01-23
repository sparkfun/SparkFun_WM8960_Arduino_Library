/******************************************************************************
  Example _05_Loopback.ino
  Demonstrates analog audio input (on INPUT1s), ADC/DAC Loopback, sets volume control, and Headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT1" inputs, 
  they are labeled "RIN1" and "LIN1" on the board.

  This example will pass your audio source through the mixers and gain stages of the codec 
  into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
  Then send the output of the DAC to the headphone outs.

  You can now control the volume of the codecs built in headphone amp using this fuction:

  codec.setHeadphoneVolume(120); Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB

  Development platform used:
  SparkFun ESP32 IoT Redboard v10

  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed for source of codec's onboard AVDD (3.3V vreg)

  **********************
  CODEC ------- AUDIO IN
  **********************
  GND --------- TRS INPUT SLEEVE        *ground connection for line level input via TRS breakout
  LINPUT1 ----- TRS INPUT TIP           *left audio
  RINPUT1 ----- TRS INPUT RING1         *right audio

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" connection for headphone output (aka "HP GND")
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


  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_WM8960
WM8960 codec;

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 5 - Loopback");

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

  // setup signal flow to the ADC

  codec.enableLMIC();
  codec.enableRMIC();
  
  // connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
  codec.connectLMN1();
  codec.connectRMN1();

  // disable mutes on PGA inputs (aka INTPUT1)
  codec.disableLINMUTE();
  codec.disableRINMUTE();

  // set input boosts to get inputs 1 to the boost mixers
  codec.setLMICBOOST(0); // 0 = 0dB
  codec.setRMICBOOST(0); // 0 = 0dB

  codec.connectLMIC2B();
  codec.connectRMIC2B();

  // enable boost mixers
  codec.enableAINL();
  codec.enableAINR();

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
  codec.setPLLPRESCALE(PLLPRESCALE_DIV_2);
  codec.setSMD(PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(CLKSEL_PLL);
  codec.setSYSCLKDIV(SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h	
  //codec.setADCDIV(0); // default is 000 (what we need for 44.1KHz), so no need to write this.
  //codec.setDACDIV(0); // default is 000 (what we need for 44.1KHz), so no need to write this.

  codec.enableMasterMode(); 
  codec.setALRCGPIO(); // note, should not be changed while ADC is enabled.

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

  Serial.println("Example complete. Listen to left/right INPUT1 on Headphone outputs.");
}

void loop()
{
  // nothing to see here.
}