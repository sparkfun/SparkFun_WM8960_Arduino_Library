/******************************************************************************
  Example _02_INPUT2.ino
  Demonstrates analog audio input (on INPUT2s), sets volume control, and headphone output on the WM8960 Codec.

  Audio should be connected to both the left and right "INPUT2" inputs, 
  they are labeled "RIN2" and "LIN2" on the board.

  This example will pass your audio source through the mixers and gain stages of the codec 
  using all of the analog bypass paths.

  It will output the sound on the headphone outputs. 
  It is setup to do a capless headphone setup, so connet your headphones ground to "OUT3" 
  and this provides a buffered VMID.

  You can now control the volume of the codecs built in headphone buffers using this fuction:

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
  Serial.println("Example 2 - INPUT2");

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

  // setup signal flow through the analog audio bypass connections

  // set input boosts to get INPUT2 (both left and right) to the boost mixers
  codec.set_LIN2BOOST(5); // 5 = 0dB
  codec.set_RIN2BOOST(5); // 5 = 0dB

  // enable input boost mixers
  codec.enable_AINL();
  codec.enable_AINR();

  // connect LB2LO (booster to output mixer [aka analog bypass])
  codec.enableLB2LO();
  codec.enableRB2RO();

  // set gainstage between boost mixer and output mixers (analog bypass)
  codec.setLB2LOVOL(7); // 7 = 0dB
  codec.setRB2ROVOL(7); // 7 = 0dB

  // enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();
  
  codec.enableHeadphones();
  codec.enableOUT3MIX(); // provides VMID as buffer for headphone ground

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolume(120);

  Serial.println("Example complete. Listen to inputs 2 on headphone outputs.");
}

void loop()
{
  // nothing to see here.
}