/******************************************************************************
  SparkFun WM8960 Arduino Library

  This library provides a set of functions to control (via I2C) the Wolfson Microelectronics WM8960
	Stereo CODEC with 1W Stereo Class D Speaker Drivers and Headphone Drivers.

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
	
	For functions that return a value, e.g. "readGain(char *gain)", first
	declare	a variable of the proper type e.g. "char x", then pass the
	address of your variable to the function by putting '&' in front of it
	e.g. "readGain(&x)". The function will modify the variable directly.

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

#include <SparkFun_WM8960_Arduino_Library.h>

WM8960::WM8960() {
  // constructor does nothing, must call Wire.begin() from within setup()
}

boolean WM8960::begin(TwoWire &wirePort)
{
  	_i2cPort = &wirePort;
  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 
  	return (true); //We're all setup!
}

//Returns true if I2C device ack's
boolean WM8960::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

// writeRegister(uint8_t reg, uint16_t value)
// General-purpose write to a register
// Returns 1 if successful, 0 if something failed (I2C error)
// The WM8960 has 9 bit registers.
// To write a register, we must do the following
// send 3 bytes
// byte0 = device address + read/write bit
// control_byte_1 = register-to-write address (7-bits) plus 9th bit of data
// control_byte_2 = remaining 8 bits of register data
boolean WM8960::writeRegister(uint8_t reg, uint16_t value)
{
  uint8_t result;
  uint8_t control_byte_1 = (reg << 1); // shift reg over one spot to make room for the 9th bit of register data
  control_byte_1 |= (value >> 8); // shift value so only 9th bit is still there, plug it into 0th bit of control_byte_1
  uint8_t control_byte_2 = (uint8_t)(value & 0xFF);
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);// I2C address (use 7-bit address, wire library will modify for read/write)
  _i2cPort->write(control_byte_1);                       // register to write + 9th bit of data
  _i2cPort->write(control_byte_2);                     // reamining 8 bits of data
  result = _i2cPort->endTransmission();
  if (result == 0)
     return 1;
  return 0;
}

// enableVREF
// Necessary for all other functions of the CODEC
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
boolean WM8960::enableVREF()
{
    uint16_t regvalue = _registerLocalCopy[WM8960_REG_PWR_MGMT_1]; // Get the local copy of the register
    regvalue |= B00100000; // modify only the bit we want  
    if (WM8960::writeRegister(WM8960_REG_PWR_MGMT_1, regvalue)); // write modified value to device
    {
        _registerLocalCopy[WM8960_REG_PWR_MGMT_1] = regvalue; // if successful, update local copy
        return 1;
    }
  return 0;
}

// disableVREF
// Use this to save power
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
boolean WM8960::disableVREF()
{
    uint16_t regvalue = _registerLocalCopy[WM8960_REG_PWR_MGMT_1]; // Get the local copy of the register
    regvalue &= B11011111; // modify only the bit we want
    if (WM8960::writeRegister(WM8960_REG_PWR_MGMT_1, regvalue)); // write modified value to device
    {
        _registerLocalCopy[WM8960_REG_PWR_MGMT_1] = regvalue; // if successful, update local copy
        return 1;
    }
  return 0;
}