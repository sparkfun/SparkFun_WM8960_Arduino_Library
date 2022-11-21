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

// writeRegisterBit
// writes a 0 or 1 to the desired bit in the desired register
boolean WM8960::_writeRegisterBit(uint8_t registerAddress, uint8_t bitNumber, boolean bitValue)
{
    uint16_t regvalue = _registerLocalCopy[registerAddress]; // Get the local copy of the register
    if(bitValue == 1) 
    {
      regvalue |= (1<<bitNumber); // set only the bit we want  
    }
    else {
      regvalue &= ~(1<<bitNumber); // clear only the bit we want  
    }
    if (WM8960::writeRegister(registerAddress, regvalue)) // write modified value to device
    {
        _registerLocalCopy[registerAddress] = regvalue; // if successful, update local copy
        return 1;
    }
  return 0;
}

// writeRegisterMultiBits
// This function writes data into the desired bits within the desired register
// Some settings require more than just flipping a single bit within a register.
// For these settings use this more advanced register write helper function.
// 
// For example, to change the LIN2BOOST setting to +6dB,
// I need to write a setting of 7 (aka +6dB) to the bits [3:1] in the WM8960_REG_INPUT_BOOST_MIXER_1 register. Like so...
//  _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 3, 1, 7);
//
boolean WM8960::_writeRegisterMultiBits(uint8_t registerAddress, uint8_t settingMsbNum, uint8_t settingLsbNum, uint8_t setting)
{
  uint8_t numOfBits = (settingMsbNum - settingLsbNum) + 1;
  uint16_t regvalue = _registerLocalCopy[registerAddress]; // Get the local copy of the register
  for(int i = 0 ; i < numOfBits ; i++)
  {
      regvalue &= ~(1 << (settingLsbNum + i)); // clear bits we care about 
  }
  regvalue |= (setting << settingLsbNum); // shift and set the bits from in incoming desired setting value
  if (WM8960::writeRegister(registerAddress, regvalue)) // write modified value to device
  {
      _registerLocalCopy[registerAddress] = regvalue; // if successful, update local copy
      return 1;
  }
  return 0;
}

// enableVREF
// Necessary for all other functions of the CODEC
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
boolean WM8960::enableVREF()
{ 
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 1);
}

// disableVREF
// Use this to save power
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
boolean WM8960::disableVREF()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 0);
}

boolean WM8960::enable_AINL()
{ 
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 1);
}

boolean WM8960::disable_AINL()
{ 
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 0);
}

boolean WM8960::enable_AINR()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 1);
}

boolean WM8960::disable_AINR()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 0);
}

boolean WM8960::enable_LMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

boolean WM8960::disable_LMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

boolean WM8960::enable_RMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

boolean WM8960::disable_RMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
}

boolean WM8960::enable_LMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

boolean WM8960::disable_LMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

boolean WM8960::enable_RMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

boolean WM8960::disable_RMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
}

		// PGA input signal select
		// Each PGA (left and right) has a switch on its non-inverting input.
		// On PGA_LEFT:
		// 	*You can select between VMID, LINPUT2 or LINPUT3
		// 	*Note, the inverting input of PGA_LEFT is perminantly connected to LINPUT1
		// On PGA_RIGHT:
		//	*You can select between VMIN, RINPUT2 or RINPUT3
		// 	*Note, the inverting input of PGA_RIGHT is perminantly connected to RINPUT1

 // 3 options: PGAL_LINPUT2, PGAL_LINPUT3, PGAL_VMID
boolean WM8960::pgaLeftNonInvSignalSelect(uint8_t signal)
{
  return true;
}

 // 3 options: PGAR_RINPUT2, PGAR_RINPUT3, PGAR_VMID
boolean WM8960::pgaRightNonInvSignalSelect(uint8_t signal)
{
  return true;
}

// Connection from each INPUT1 to the inverting input of its PGA
boolean WM8960::connect_LMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 1);
}

// Disconnect LINPUT1 to inverting input of Left Input PGA
boolean WM8960::disconnect_LMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 0);
}

// Connect RINPUT1 from inverting input of Right Input PGA
boolean WM8960::connect_RMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 1);
}

// Disconnect RINPUT1 to inverting input of Right Input PGA
boolean WM8960::disconnect_RMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 0);
}

// Connections from output of PGAs to downstream "boost mixers".

// Connect Left Input PGA to Left Input Boost mixer
boolean WM8960::connect_LMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 1);
}

// Disconnect Left Input PGA to Left Input Boost mixer
boolean WM8960::disconnect_LMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 0);
}

// Connect Right Input PGA to Right Input Boost mixer
boolean WM8960::connect_RMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 1);
}

// Disconnect Right Input PGA to Right Input Boost mixer
boolean WM8960::disconnect_RMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 0);
}

boolean WM8960::set_LINVOL(uint8_t volume) // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
{
  if(volume >= 63) volume = 63; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_INPUT_VOLUME,5,0,volume);
  boolean result2 = WM8960::pgaLeftIPVUSet();
  return (result1 && result2);
}

boolean WM8960::set_RINVOL(uint8_t volume) // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
{
  if(volume >= 63) volume = 63; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_INPUT_VOLUME,5,0,volume);
  boolean result2 = WM8960::pgaRightIPVUSet();
  return (result1 && result2);
}

// Zero Cross prevents zipper sounds on volume changes
// sets both left and right PGAs
boolean WM8960::pgaZeroCrossOn()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 1) == 0) return 0;
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1);
}

boolean WM8960::pgaZeroCrossOff()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 0) == 0) return 0;
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0);
}

boolean WM8960::enable_LINMUTE()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 1);
}

boolean WM8960::disable_LINMUTE()
{
  WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 0);
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

boolean WM8960::enable_RINMUTE()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1);
}

boolean WM8960::disable_RINMUTE()
{
  WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 0);
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}

// causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
boolean WM8960::pgaLeftIPVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

 // causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
boolean WM8960::pgaRightIPVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}


// Input Boosts

boolean WM8960::set_LMICBOOST(uint8_t boost_gain) // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
{
  if(boost_gain >= 3) boost_gain = 3; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCL_SIGNAL_PATH,5,4,boost_gain);
}
boolean WM8960::set_RMICBOOST(uint8_t boost_gain) // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
{
  if(boost_gain >= 3) boost_gain = 3; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCR_SIGNAL_PATH,5,4,boost_gain);
}
boolean WM8960::set_LIN3BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain >= 7) boost_gain = 7; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,6,4,boost_gain);
}
boolean WM8960::set_LIN2BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain >= 7) boost_gain = 7; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,3,1,boost_gain);
}
boolean WM8960::set_RIN3BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain >= 7) boost_gain = 7; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,6,4,boost_gain);
}
boolean WM8960::set_RIN2BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB	
{
  if(boost_gain >= 7) boost_gain = 7; // limit incoming values max
  if(boost_gain <= 0) boost_gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,3,1,boost_gain);
}

// Mic Bias control
boolean WM8960::enableMicBias()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 1);
}

boolean WM8960::disableMicBias()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 0);
}

// MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
boolean WM8960::setMicBiasVoltage(boolean voltage)
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 0, voltage);
}

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ADC
		/////////////////////////////////////////////////////////

boolean WM8960::enableAdcLeft()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 1);
}

boolean WM8960::disableAdcLeft()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 0);
}

boolean WM8960::enableAdcRight()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 1);
}

boolean WM8960::disableAdcRight()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 0);
}

/*

		// ADC digital volume
		// note, also needs to handle control of the ADCVU bits (volume update).
		// valid inputs are 0-255
		// 0 = mute
		// 1 = -97dB
		// ... 0.5dB steps up to
		// 255 = +30dB
boolean WM8960::setAdcLeftDigitalVolume(uint8_t volume); 
boolean WM8960::setAdcRightDigitalVolume(uint8_t volume);

*/
		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ALC
		/////////////////////////////////////////////////////////



		// Automatic Level Control
		// Note that when the ALC function is enabled, the settings of
		// registers 0 and 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and RINMUTE) are ignored.
boolean WM8960::enableAlc(uint8_t mode)
{
  boolean bit8 = (mode>>1);
  boolean bit7 = (mode & B00000001);
  if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, bit8) == 0) return 0;
  return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, bit7);
}

 // also sets alc sample rate to match global sample rate.
boolean WM8960::disableAlc()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, 0) == 0) return 0;
  return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, 0);
}

/*

boolean WM8960::setAlcTarget(uint8_t target); // valid inputs are 0-15
boolean WM8960::setAlcDecay(uint8_t decay); // valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
boolean WM8960::setAlcAttack(uint8_t attack); // valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds

*/

		// Peak Limiter
boolean WM8960::enablePeakLimiter()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ALC3, 8, 1);
}

boolean WM8960::disablePeakLimiter()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ALC3, 8, 0);
}

		// Noise Gate
boolean WM8960::enableNoiseGate()
{
  return WM8960::_writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 1);
}

boolean WM8960::disableNoiseGate()
{
  return WM8960::_writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 0);
}

boolean WM8960::setNoiseGateThreshold(uint8_t threshold) // 0-31, 0 = -76.5dBfs, 31 = -30dBfs
{
  return true;
}

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// DAC
		/////////////////////////////////////////////////////////

		// enable/disble each channel
boolean WM8960::enableDacLeft()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 1);
}

boolean WM8960::disableDacLeft()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 0);
}

boolean WM8960::enableDacRight()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 1);
}

boolean WM8960::disableDacRight()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 0);
}

/*

		// DAC digital volume
		// note, also needs to handle control of the DACVU bits (volume update).
		// valid inputs are 0-255
		// 0 = mute
		// 1 = -127dB
		// ... 0.5dB steps up to
		// 255 = 0dB
boolean WM8960::setDacLeftDigitalVolume(uint8_t volume); 
boolean WM8960::setDacRightDigitalVolume(uint8_t volume);		

*/

		// DAC mute
boolean WM8960::enableDacMute()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 1);
}

boolean WM8960::disableDacMute()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
}






		// DE-Emphasis





// 3D Stereo Enhancement
// 3D enable/disable
boolean WM8960::enable3d()
{
  return WM8960::_writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 1);
}

boolean WM8960::disable3d()
{
  return WM8960::_writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 0);
}

boolean WM8960::set3dDepth(uint8_t depth) // 0 = 0%, 15 = 100%
{
  if(depth >= 15) depth = 15; // limit incoming values max
  if(depth <= 0) depth = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_3D_CONTROL,4,1,depth);
}



		// 3D upper/lower cut-off frequencies.



// DAC output -6dB attentuation enable/disable
boolean WM8960::enableDac6dbAttenuation()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 1);
}

boolean WM8960::disableDac6dbAttentuation()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 0);
}

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// OUTPUT mixers
		/////////////////////////////////////////////////////////

		// what's connected to what? Oh so many options...
		// LOMIX	Left Output Mixer
		// ROMIX	Right Output Mixer
		// OUT3MIX		Mono Output Mixer

		// enable/disable left and right output mixers
boolean WM8960::enableLOMIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 1);
}

boolean WM8960::disableLOMIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 0);
}

boolean WM8960::enableROMIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 1);
}

boolean WM8960::disableROMIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 0);
}

boolean WM8960::enableOUT3MIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 1);
}

boolean WM8960::disableOUT3MIX()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 0);
}

// enable/disable audio path connections/vols to/from output mixers
// see datasheet page 35 for a nice image of all the connections.
boolean WM8960::enableLI2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 1);
}

boolean WM8960::disableLI2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 0);
}

boolean WM8960::setLI2LOVOL(uint8_t volume) // 0-7, 0 = -21dB, ... 3dB steps ... 7 = 0dB
{
  if(volume >= 7) volume = 7; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  volume = 7 - volume; // flip it so 0 = lowest volume and 7 = highest volume
  return WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_OUT_MIX_1,6,4,volume);
}

boolean WM8960::enableLB2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_1, 7, 1);
}

boolean WM8960::disableLB2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_1, 7, 0);
}

boolean WM8960::setLB2LOVOL(uint8_t volume) // 0-7, 0 = -21dB, ... 3dB steps ... 7 = 0dB
{
  if(volume >= 7) volume = 7; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  volume = 7 - volume; // flip it so 0 = lowest volume and 7 = highest volume
  return WM8960::_writeRegisterMultiBits(WM8960_REG_BYPASS_1,6,4,volume);
}

boolean WM8960::enableLD2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 1);
}

boolean WM8960::disableLD2LO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 0);
}

boolean WM8960::enableRI2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
}

boolean WM8960::disableRI2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 0);
}

boolean WM8960::setRI2ROVOL(uint8_t volume) // 0-7, 0 = -21dB, ... 3dB steps ... 7 = 0dB
{
  if(volume >= 7) volume = 7; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  volume = 7 - volume; // flip it so 0 = lowest volume and 7 = highest volume
  return WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_OUT_MIX_2,6,4,volume);
}

boolean WM8960::enableRB2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_2, 7, 1);
}

boolean WM8960::disableRB2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_2, 7, 0);
}

boolean WM8960::setRB2ROVOL(uint8_t volume) // 0-7, 0 = -21dB, ... 3dB steps ... 7 = 0dB
{
  if(volume >= 7) volume = 7; // limit incoming values max
  if(volume <= 0) volume = 0; // limit incoming values min
  volume = 7 - volume; // flip it so 0 = lowest volume and 7 = highest volume
  return WM8960::_writeRegisterMultiBits(WM8960_REG_BYPASS_2,6,4,volume);
}

boolean WM8960::enableRD2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
}


boolean WM8960::disableRD2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 0);
}

		// Mono Output mixer. 
		// note, for capless HPs, we'll want this to output a buffered VMID.
		// to do this, we need to disable both of these connections.
boolean WM8960::enableLI2MO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 1);
}

boolean WM8960::disableLI2MO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 0);
}

boolean WM8960::enableRI2MO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 1);
}

boolean WM8960::disableRI2MO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 0);
}

/*

// this will disable both connections, thus enable VMID on OUT3
// note, to enable VMID, you also need to enable OUT3 in the WM8960_REG_PWR_MGMT_2 [1]
// see next funtion
boolean WM8960::enableOUT3asVMID()
{
  return WM8960::_writeRegisterBit(WM8960_REG_HOLDER, 6, 0);
}

*/

// enables VMID in the WM8960_REG_PWR_MGMT_2 register, and set's it to playback/record settings of 2*50Kohm.
boolean WM8960::enableVMID()
{
  WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 8, 1);
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 7, 1);
}

 
boolean WM8960::disableVMID()
{
  WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 8, 0);
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 7, 0);
}



/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Headphones
/////////////////////////////////////////////////////////

// Enable and disable headphones (mute)
boolean WM8960::enableHeadphones()
{
  return (WM8960::enableRightHeadphone() & WM8960::enableLeftHeadphone());
}

boolean WM8960::disableHeadphones()
{
  return (WM8960::disableRightHeadphone() & WM8960::disableLeftHeadphone());
}

boolean WM8960::enableRightHeadphone()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 1);
}

boolean WM8960::disableRightHeadphone()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 0);
}

boolean WM8960::enableLeftHeadphone()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 1);
}

boolean WM8960::disableLeftHeadphone()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 0);
}

boolean WM8960::enableHeadphoneStandby()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 1);
}

boolean WM8960::disableHeadphoneStandby()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 0);
}

// setHeadphoneVolume
// Sets the volume for both left and right headphone outpus
// 
// Although you can control each headphone output independently, here we are
// going to assume you want both left and right to do the same thing.
// 
boolean WM8960::setHeadphoneVolume(uint8_t volume) // Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
{		
  // updates both left and right channels
	// handles the OUT1VU (volume update) bit control, so that it happens at the same time on both channels.
	// Note, we must also make sure that the outputs are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
  // grab local copy of register
  // modify the bits we need to
  // write register in device, including the volume update bit write
  // if successful, save locally.

  // limit inputs
  if (volume >= 127) volume = 127;
  if (volume <=0) volume = 0;

  // LEFT
  //boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT1_VOLUME,6,0,volume);
    uint16_t regvalue = _registerLocalCopy[WM8960_REG_LOUT1_VOLUME]; // Get the local copy of the register
    regvalue &= (B10000000); // clear bits we care about [6:0] are LOUT1VOL

    regvalue |= volume; // set the bits from in incoming desired volume value

    boolean result1 = WM8960::writeRegister(WM8960_REG_LOUT1_VOLUME, regvalue); // write register
    if(result1) 
    {
      _registerLocalCopy[WM8960_REG_LOUT1_VOLUME] = regvalue; // if successful, update local copy
    }

  // RIGHT
  //boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT1_VOLUME,6,0,volume);
    regvalue = _registerLocalCopy[WM8960_REG_ROUT1_VOLUME]; // Get the local copy of the register
    regvalue &= (B10000000); // clear bits we care about [6:0] are LOUT1VOL

    regvalue |= volume; // set the bits from in incoming desired volume value

    boolean result2 = WM8960::writeRegister(WM8960_REG_ROUT1_VOLUME, regvalue); // write register
    if(result2) 
    {
      _registerLocalCopy[WM8960_REG_ROUT1_VOLUME] = regvalue; // if successful, update local copy
    }

    boolean result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 8, 1); // updated left channel
    boolean result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 8, 1); // updated right channel

    if (result1 && result2 && result3 && result4) // if all I2C sommands Ack'd, then...
    {
        return 1;
    }
  return 0;
}

/*

		// Zero Cross prevents zipper sounds on volume changes
boolean WM8960::headphoneZeroCrossOn()
{
  return WM8960::_writeRegisterBit(WM8960_REG_HOLDER, 6, 0);
}

 // sets both left and right Headphone outputs
boolean WM8960::headphoneZeroCrossOff()
{
  return WM8960::_writeRegisterBit(WM8960_REG_HOLDER, 6, 0);
}

 // sets both left and right Headphone outputs
		
*/
		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Speakers
		/////////////////////////////////////////////////////////

		// Enable and disable speakers (mute)
boolean WM8960::enableSpeakers()
{
  return (WM8960::enableRightSpeaker() & WM8960::enableLeftSpeaker());
}

boolean WM8960::disableSpeakers()
{
  return (WM8960::disableRightHeadphone() & WM8960::disableLeftHeadphone());
}

boolean WM8960::enableRightSpeaker()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 1); //SPK_OP_EN
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 1); // SPKR
  return (result1 & result2);
}

boolean WM8960::disableRightSpeaker()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 0); //SPK_OP_EN
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 0); // SPKR
  return (result1 & result2);
}

boolean WM8960::enableLeftSpeaker()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 1); //SPK_OP_EN
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 1); // SPKL
  return (result1 & result2);
}

boolean WM8960::disableLeftSpeaker()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 0); //SPK_OP_EN
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 0); // SPKL
  return (result1 & result2);
}

// setSpeakerVolume
// sets to volume for both left and right speaker outputs
// 
// Although you can control each Speaker output independently, here we are
// going to assume you want both left and right to do the same thing.
// 
boolean WM8960::setSpeakerVolume(uint8_t volume) // Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
{		
  // updates both left and right channels
	// handles the SPKVU (volume update) bit control, so that it happens at the same time on both channels.
	// Note, we must also make sure that the outputs are enabled in the WM8960_REG_PWR_MGMT_2 [4:3]
	// and the class D control reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

  // limit inputs
  if (volume >= 127) volume = 127;
  if (volume <=0) volume = 0;

  // LEFT
    boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT2_VOLUME,6,0,volume);
  // RIGHT
    boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT2_VOLUME,6,0,volume);
  // SPKVU
    boolean result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 8, 1); // updated left channel
    boolean result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 8, 1); // updated right channel

    if (result1 && result2 && result3 && result4) // if all I2C sommands Ack'd, then...
    {
        return 1;
    }
  return 0;
}

// Zero Cross prevents zipper sounds on volume changes
// sets both left and right Speaker outputs
boolean WM8960::enableSpeakerZeroCross()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1);
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1); // left
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 1); // right
  return (result1 & result2);
}

boolean WM8960::disableSpeakerZeroCross()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1);
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 0); // left
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 0); // right
  return (result1 & result2);
}

	
// setSpeakerDcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
boolean WM8960::setSpeakerDcGain(uint8_t gain)
{
  if(gain >= 5) gain = 5; // limit incoming values max
  if(gain <= 0) gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,5,3,gain);
}

// setSpeakerAcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
boolean WM8960::setSpeakerAcGain(uint8_t gain)
{
  if(gain >= 5) gain = 5; // limit incoming values max
  if(gain <= 0) gain = 0; // limit incoming values min
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,2,0,gain);
}


		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Digital audio interface control
		/////////////////////////////////////////////////////////

		// defaults to I2S, peripheral-mode, 24-bit word length
		// *Might need to change the WL to match our application. 
		// *I believe the BT example will need 16-bit WL.
boolean WM8960::setAudioDataWordLength(uint8_t length) // 0=16bit, 1=20bit, 2=24bit, 3=32bit.
{
  return 1;
}

// Loopback
// When enabled, the output data from the ADC audio interface is fed directly into the DAC data input.
boolean WM8960::enableLoopBack()
{
  return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 1);
}

boolean WM8960::disableLoopBack()
{
  return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 0);
}



		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Clock controls
		/////////////////////////////////////////////////////////

		// Getting the Frequency of SampleRate as we wish
		// Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
		// According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a SR of 44.1KHz
		// To get that Desired Output (SYSCLK), we need the following settings on the PLL stuff.
		// as found on table 45 (ds pg 61).
		// PRESCALE DIVIDE (PLLPRESCALE): 2
		// POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
		// FIXED POST-DIVIDE: 4
		// R: 7.5264 
		// N: 7h
		// K: 86C226h

		// example at bottom of table 46, shows that we should be in fractional mode for a 44.1KHz.

		// In terms of registers, this is what we want for 44.1KHz
		// PLLEN=1			(PLL enable)
		// PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down to 12MHZ for F2
		// PLLN=7h			(PLL N value) *this is "int R"
		// PLLK=86C226h		(PLL K value) *this is int ( 2^24 * (R- intR)) 
		// SDM=1			(Fractional mode)
		// CLKSEL=1			(PLL select) 
		// MS=0				(Peripheral mode)
		// WL=00			(16 bits)
		// SYSCLKDIV=2		(Divide by 2)
		// ADCDIV=000		(Divide by 1) = 44.1kHz
		// DACDIV=000		(Divide by 1) = 44.1kHz
		// BCLKDIV=0100		(Divide by 4) = 64fs
		// DCLKDIV=111		(Divide by 16) = 705.6kHz

		// And now for the functions that will set these registers...
boolean WM8960::enablePLL()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 1);
}


boolean WM8960::disablePLL()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 0);
}


boolean WM8960::set_PLLPRESCALE(boolean div)
{
  return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 4, div);
}

boolean WM8960::set_PLLN(uint8_t n)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_N,3,0,n); 
}

boolean WM8960::set_PLLK(uint8_t one, uint8_t two, uint8_t three) // send each nibble of 24-bit value for value K
{
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_1,5,0,one); 
  boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_2,8,0,two); 
  boolean result3 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_3,8,0,three); 
  if (result1 && result2 && result3) // if all I2C sommands Ack'd, then...
  {
    return 1;
  }
  return 0;  
}

boolean WM8960::set_SMD(boolean mode)
{
  return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 5, mode);
}

 // 0=integer, 1=fractional
boolean WM8960::set_CLKSEL(boolean sel)
{
  return WM8960::_writeRegisterBit(WM8960_REG_CLOCKING_1, 0, sel);
}

 // 0=MCLK, 1=PLL_output
boolean WM8960::set_SYSCLKDIV(uint8_t div) // (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,2,1,div);  
}

//boolean WM8960::set_ADCDIV(uint8_t setting); // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options

//boolean WM8960::set_DACDIV(uint8_t setting); // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options

boolean WM8960::set_BCLKDIV(uint8_t div)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,3,0,div);  
}

boolean WM8960::set_DCLKDIV(uint8_t setting) // Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,8,6,setting);
}

boolean WM8960::set_ALRCGPIO()
{
  // This setting should not be changed if ADCs are enabled.
  return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 6, 1);
}

boolean WM8960::enableMasterMode()
{
  return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 1);
}

boolean WM8960::enablePeripheralMode()
{
  return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 0);
}

boolean WM8960::set_WL(uint8_t word_length)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_AUDIO_INTERFACE_1,3,2,word_length);  
}