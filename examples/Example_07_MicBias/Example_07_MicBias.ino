/******************************************************************************
  Example _07_MicBias.ino
  Demonstrates Mic Bias feature of WM8960 Codec.

  Turns on the micBias, set's it to each available output voltage,
  Then turns it off to demonstrate disable.

  Development platform used:
  SparkFun ESP32 IoT Redboard v10

  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed for source of codec's onboard AVDD (3.3V vreg)

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
  Serial.println("Example 7 - MicBias Control");

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

  codec.enableMicBias();
  codec.setMicBiasVoltage(MIC_BIAS_VOLTAGE_0_9_AVDD); // MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
  Serial.println("Mic Bias enabled (0.9*AVDD)");
  delay(3000);

  codec.setMicBiasVoltage(MIC_BIAS_VOLTAGE_0_65_AVDD); // MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
  Serial.println("Mic Bias enabled (0.65*AVDD)");
  delay(3000);

  codec.disableMicBias();
  Serial.println("Mic Bias disabled");
  
  Serial.println("Example Complete. Hit reset to begin again.");
}

void loop()
{
  // nothing to see here
}