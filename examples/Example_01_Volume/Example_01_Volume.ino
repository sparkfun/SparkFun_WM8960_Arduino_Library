/******************************************************************************
  Example _01_Volume.ino
  Demonstrates analog audio input, volume control, and headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT3" inputs, 
  they are labeled "RIN3" and "LIN3" on the board.

  This example will pass your audio source through the mixers and gain stages of the codec 
  using all of the analog bypass paths.

  It will output the sound on the headphone outputs. 
  It is setup to do a capless headphone setup, so connet your headphones ground to "OUT3" 
  and this provides a buffered VMID.

  You can now control the volume of the codecs built in headphone buffers using this fuction:

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
  LINPUT3 ----- TRS INPUT TIP           *left audio
  RINPUT3 ----- TRS INPUT RING1         *right audio

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
  codec.enableLOMIX(); // Enable left output mixer
  codec.enableLI2LO(); // Enable bypass connection from Left INPUT3 to Left output mixer, note, the default gain on this input (LI2LOVOL) is -15dB
  codec.setLI2LOVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); // Sets volume control between "left input" to "left output mixer"
  
  codec.enableROMIX(); // Now for the right channel of INPUT3
  codec.enableRI2RO();
  codec.setRI2ROVOL(WM8960_OUTPUT_MIXER_GAIN_0DB);
  
  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  Serial.println("Volume set to +6dB (max)");
  codec.setHeadphoneVolume(127); // 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
  delay(5000);

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolume(120);
  delay(5000);

  Serial.println("Volume set to -6dB");
  codec.setHeadphoneVolume(114);
  delay(5000);  

  Serial.println("Volume set to mute");
  codec.setHeadphoneVolume(47);
  delay(5000);

  Serial.println("Example complete. Hit Reset to try again.");
}

void loop()
{
  // Nothing to see here.
}