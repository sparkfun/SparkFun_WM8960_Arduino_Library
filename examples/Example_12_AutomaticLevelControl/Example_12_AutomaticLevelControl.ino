/******************************************************************************
  Example_12_AutomaticLevelControl.ino
  Demonstrates how to use the automatic level control feature of the WM8960 Codec.

  Attach a potentiomenter to GND/A0/3V3 to actively adjust the ALC target setting.

  This example sets up the codec for analog audio input (on INPUT1s), ADC/DAC Loopback, sets hp volume, and Headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT1" inputs, 
  they are labeled "RIN1" and "LIN1" on the board.

  This example will pass your audio source through the mixers and gain stages of the codec 
  into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
  Then send the output of the DAC to the headphone outs.

  We will use the user input via potentiometer on A0 to set the ALC target value.
  The ALC will adjust the gain of the pga input buffer to try and keep the signal level at the target.

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
  ESP32 -------- POTENTIOMTER (aka blue little trimpot)
  **********************
  GND ---------- "right-side pin"          
  A0 ----------- center pin            *aka center tap connection
  3V3 ---------- "left-side pin"             

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

long userInputA0 = 0; // Used to store incoming potentiometer settings to set ADC digital volume setting

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 12 - Automatic Level Control");

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
  for (int i = 0 ; i < 250 ; i ++) // Take a bunch of readings and average them, to smooth out the value
  {
    userInputA0 += analogRead(A0);
    delay(1);
  }
  userInputA0 /= 250;

  // Map it from 0-4096, to a value that is acceptable for the setting
  int alcTarget = map(userInputA0, 0, 4096, 15, 0);

  Serial.print("alcTarget: ");
  Serial.println(alcTarget);
  
  codec.setAlcTarget(alcTarget); // Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS

  delay(1000);
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

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d freq at 705.6kHz
  codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h	
  //codec.setADCDIV(0); // Default is 000 (what we need for 44.1KHz), so no need to write this.
  //codec.setDACDIV(0); // Default is 000 (what we need for 44.1KHz), so no need to write this.

  codec.enableMasterMode(); 
  codec.setALRCGPIO(); // Note, should not be changed while ADC is enabled.

  // Enable ADCs and DACs
  codec.enableAdcLeft();
  codec.enableAdcRight();
  codec.enableDacLeft();
  codec.enableDacRight();
  codec.disableDacMute();

  codec.enableLoopBack(); // Loopback sends ADC data directly into DAC
  codec.disableDacMute(); // Default is "soft mute" on, so we must disable mute to make channels active

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  // Automatic Level control stuff
  codec.enablePgaZeroCross(); // Only allows pga gain stages at a "zero crossover" point in audio stream. 
  // Minimizes "zipper" noise when chaning gains.
  codec.enableAlc(WM8960_ALC_MODE_STEREO);
  codec.setAlcTarget(11); // Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS
  codec.setAlcDecay(3); // Valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
  codec.setAlcAttack(2); // Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds
  codec.setAlcMaxGain(7); // Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
  codec.setAlcMinGain(0); // Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
  codec.setAlcHold(0); // Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s


  Serial.println("Headphopne Amp Volume set to +0dB");
  codec.setHeadphoneVolumeDB(0.00);

  Serial.println("Codec setup complete. Listen to left/right INPUT1 on Headphone outputs.");
}