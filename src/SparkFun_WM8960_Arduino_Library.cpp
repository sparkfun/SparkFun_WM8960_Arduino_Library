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
    if (reset() == false) // reset all registers to default values
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
     return true;
  return false;
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
        return true;
    }
  return false;
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
      return true;
  }
  return false;
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

// reset
// Use this to reset all registers to their default state
// Note, this can also be done by cycling power to the device
// Returns 1 if successful, 0 if something failed (I2C error)
boolean WM8960::reset()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_RESET, 7, 1)) // doesn't really matter which bit we flip, writing anything will cause the reset
  {
    // update our local copy of the registers to reflect the reset
    for(int i = 0 ; i < 56 ; i++)
    {
      _registerLocalCopy[i] = _registerDefaults[i]; 
    }
    return true;
  }
  return false;
}

boolean WM8960::enableAINL()
{ 
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 1);
}

boolean WM8960::disableAINL()
{ 
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 0);
}

boolean WM8960::enableAINR()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 1);
}

boolean WM8960::disableAINR()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 0);
}

boolean WM8960::enableLMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

boolean WM8960::disableLMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

boolean WM8960::enableRMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

boolean WM8960::disableRMIC()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
}

boolean WM8960::enableLMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

boolean WM8960::disableLMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

boolean WM8960::enableRMICBOOST()
{
  return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

boolean WM8960::disableRMICBOOST()
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

 // 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
boolean WM8960::pgaLeftNonInvSignalSelect(uint8_t signal)
{
  // clear LMP2 and LMP3
  // Necessary because the previous setting could have either set,
  // and we don't want to confuse the codec.
  // Only 1 input can be selected.
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 0); // LMP3
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 0); // LMP2
  boolean result3 = false;

  if(signal == WM8960_PGAL_LINPUT2)
  {
    result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 1); // LMP2
  }
  else if(signal == WM8960_PGAL_LINPUT3)
  {
    result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 1); // LMP3
  }
  else if(signal == WM8960_PGAL_VMID)
  {
    // don't set any bits. When both LMP2 and LMP3 are cleared, then the signal is set to VMID
  }
  return (result1 && result2 && result3);
}

 // 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
boolean WM8960::pgaRightNonInvSignalSelect(uint8_t signal)
{
  // clear RMP2 and RMP3
  // Necessary because the previous setting could have either set,
  // and we don't want to confuse the codec.
  // Only 1 input can be selected.
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 0); // RMP3
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 0); // RMP2
  boolean result3 = false;

  if(signal == WM8960_PGAR_RINPUT2)
  {
    result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 1); // RMP2
  }
  else if(signal == WM8960_PGAR_RINPUT3)
  {
    result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 1); // RMP3
  }
  else if(signal == WM8960_PGAR_VMID)
  {
    // don't set any bits. When both RMP2 and RMP3 are cleared, then the signal is set to VMID
  }
  return (result1 && result2 && result3);
}

// Connection from each INPUT1 to the inverting input of its PGA
boolean WM8960::connectLMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 1);
}

// Disconnect LINPUT1 to inverting input of Left Input PGA
boolean WM8960::disconnectLMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 0);
}

// Connect RINPUT1 from inverting input of Right Input PGA
boolean WM8960::connectRMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 1);
}

// Disconnect RINPUT1 to inverting input of Right Input PGA
boolean WM8960::disconnectRMN1()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 0);
}

// Connections from output of PGAs to downstream "boost mixers".

// Connect Left Input PGA to Left Input Boost mixer
boolean WM8960::connectLMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 1);
}

// Disconnect Left Input PGA to Left Input Boost mixer
boolean WM8960::disconnectLMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 0);
}

// Connect Right Input PGA to Right Input Boost mixer
boolean WM8960::connectRMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 1);
}

// Disconnect Right Input PGA to Right Input Boost mixer
boolean WM8960::disconnectRMIC2B()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 0);
}

boolean WM8960::setLINVOL(uint8_t volume) // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
{
  if(volume > 63) volume = 63; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_INPUT_VOLUME,5,0,volume);
  boolean result2 = WM8960::pgaLeftIPVUSet();
  return (result1 && result2);
}

boolean WM8960::setRINVOL(uint8_t volume) // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
{
  if(volume > 63) volume = 63; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_INPUT_VOLUME,5,0,volume);
  boolean result2 = WM8960::pgaRightIPVUSet();
  return (result1 && result2);
}

// Zero Cross prevents zipper sounds on volume changes
// sets both left and right PGAs
boolean WM8960::enablePgaZeroCross()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 1) == 0) return false;
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1);
}

boolean WM8960::disablePgaZeroCross()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 0) == 0) return false;
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0);
}

boolean WM8960::enableLINMUTE()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 1);
}

boolean WM8960::disableLINMUTE()
{
  WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 0);
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

boolean WM8960::enableRINMUTE()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1);
}

boolean WM8960::disableRINMUTE()
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

boolean WM8960::setLMICBOOST(uint8_t boost_gain) // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
{
  if(boost_gain > 3) boost_gain = 3; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCL_SIGNAL_PATH,5,4,boost_gain);
}
boolean WM8960::setRMICBOOST(uint8_t boost_gain) // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
{
  if(boost_gain > 3) boost_gain = 3; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCR_SIGNAL_PATH,5,4,boost_gain);
}
boolean WM8960::setLIN3BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain > 7) boost_gain = 7; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,6,4,boost_gain);
}
boolean WM8960::setLIN2BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain > 7) boost_gain = 7; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,3,1,boost_gain);
}
boolean WM8960::setRIN3BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
{
  if(boost_gain > 7) boost_gain = 7; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,6,4,boost_gain);
}
boolean WM8960::setRIN2BOOST(uint8_t boost_gain) // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB	
{
  if(boost_gain > 7) boost_gain = 7; // limit incoming values max
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

// WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
boolean WM8960::setMicBiasVoltage(boolean voltage)
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 0, voltage);
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
    // 195 = +0dB
		// 255 = +30dB
    */
boolean WM8960::setAdcLeftDigitalVolume(uint8_t volume)
{
  if(volume > 255) volume = 255; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_ADC_VOLUME,7,0,volume);
  boolean result2 = WM8960::adcLeftADCVUSet();
  return (result1 && result2);
}
boolean WM8960::setAdcRightDigitalVolume(uint8_t volume)
{
  if(volume > 255) volume = 255; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_ADC_VOLUME,7,0,volume);
  boolean result2 = WM8960::adcRightADCVUSet();
  return (result1 && result2);
}

// causes left and right input adc digital volumes to be updated
boolean WM8960::adcLeftADCVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_ADC_VOLUME, 8, 1);
}

 // causes left and right input adc digital volumes to be updated
boolean WM8960::adcRightADCVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_ADC_VOLUME, 8, 1);
}

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
  if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, bit8) == 0) return false;
  return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, bit7);
}

 // also sets alc sample rate to match global sample rate.
boolean WM8960::disableAlc()
{
  if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, 0) == 0) return false;
  return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, 0);
}

boolean WM8960::setAlcTarget(uint8_t target) // valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS
{
  if(target > 15) target = 15; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC1,3,0,target);
}

boolean WM8960::setAlcDecay(uint8_t decay) // valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
{
  if(decay > 10) decay = 10; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC3,7,4,decay);
}

boolean WM8960::setAlcAttack(uint8_t attack) // valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds
{
  if(attack > 10) attack = 10; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC3,3,0,attack);
}

boolean WM8960::setAlcMaxGain(uint8_t maxGain) // valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
{
  if(maxGain > 7) maxGain = 7; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC1,6,4,maxGain);
}

boolean WM8960::setAlcMinGain(uint8_t minGain) // valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
{
  if(minGain > 7) minGain = 7; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC2,6,4,minGain);
}

boolean WM8960::setAlcHold(uint8_t hold) // valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
{
  if(hold > 15) hold = 15; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC2,3,0,hold);
}

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

		// DAC digital volume
		// valid inputs are 0-255
		// 0 = mute
		// 1 = -127dB
		// ... 0.5dB steps up to
		// 255 = 0dB
boolean WM8960::setDacLeftDigitalVolume(uint8_t volume)
{
  if(volume > 255) volume = 255; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_DAC_VOLUME,7,0,volume);
  boolean result2 = WM8960::dacLeftDACVUSet();
  return (result1 && result2);
}
boolean WM8960::setDacRightDigitalVolume(uint8_t volume)
{
  if(volume > 255) volume = 255; // limit incoming values max
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_DAC_VOLUME,7,0,volume);
  boolean result2 = WM8960::dacRightDACVUSet();
  return (result1 && result2);
}

// causes left and right input dac digital volumes to be updated
boolean WM8960::dacLeftDACVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_LEFT_DAC_VOLUME, 8, 1);
}

 // causes left and right input dac digital volumes to be updated
boolean WM8960::dacRightDACVUSet()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_DAC_VOLUME, 8, 1);
}	

		// DAC mute
boolean WM8960::enableDacMute()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 1);
}

boolean WM8960::disableDacMute()
{
  return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
}

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
  if(depth > 15) depth = 15; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_3D_CONTROL,4,1,depth);
}

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
  if(volume > 7) volume = 7; // limit incoming values max
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
  if(volume > 7) volume = 7; // limit incoming values max
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
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 1);
}

boolean WM8960::disableRI2RO()
{
  return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 0);
}

boolean WM8960::setRI2ROVOL(uint8_t volume) // 0-7, 0 = -21dB, ... 3dB steps ... 7 = 0dB
{
  if(volume > 7) volume = 7; // limit incoming values max
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
  if(volume > 7) volume = 7; // limit incoming values max
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
  if (volume > 127) volume = 127;

  // LEFT
    boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT1_VOLUME,6,0,volume);
  // RIGHT
    boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT1_VOLUME,6,0,volume);
  // UPDATES
    boolean result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 8, 1); // updated left channel
    boolean result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 8, 1); // updated right channel

    if (result1 && result2 && result3 && result4) // if all I2C sommands Ack'd, then...
    {
        return true;
    }
  return false; 
}

		// Zero Cross prevents zipper sounds on volume changes
    // sets both left and right Headphone outputs
boolean WM8960::enableHeadphoneZeroCross()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 1); // left
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 1); // right
  return (result1 & result2);
}

boolean WM8960::disableHeadphoneZeroCross()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 0); // left
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 0); // right
  return (result1 & result2);
}

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
  if (volume > 127) volume = 127;

  // LEFT
    boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT2_VOLUME,6,0,volume);
  // RIGHT
    boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT2_VOLUME,6,0,volume);
  // SPKVU
    boolean result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 8, 1); // updated left channel
    boolean result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 8, 1); // updated right channel

    if (result1 && result2 && result3 && result4) // if all I2C sommands Ack'd, then...
    {
        return true;
    }
  return false;
}

// Zero Cross prevents zipper sounds on volume changes
// sets both left and right Speaker outputs
boolean WM8960::enableSpeakerZeroCross()
{
  boolean result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1); // left
  boolean result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 1); // right
  return (result1 & result2);
}

boolean WM8960::disableSpeakerZeroCross()
{
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
  if(gain > 5) gain = 5; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,5,3,gain);
}

// setSpeakerAcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
boolean WM8960::setSpeakerAcGain(uint8_t gain)
{
  if(gain > 5) gain = 5; // limit incoming values max
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,2,0,gain);
}

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Digital audio interface control
		/////////////////////////////////////////////////////////

		// defaults to I2S, peripheral-mode, 24-bit word length

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

boolean WM8960::setPLLPRESCALE(boolean div)
{
  return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 4, div);
}

boolean WM8960::setPLLN(uint8_t n)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_N,3,0,n); 
}

boolean WM8960::setPLLK(uint8_t one, uint8_t two, uint8_t three) // send each nibble of 24-bit value for value K
{
  boolean result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_1,5,0,one); 
  boolean result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_2,8,0,two); 
  boolean result3 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_3,8,0,three); 
  if (result1 && result2 && result3) // if all I2C sommands Ack'd, then...
  {
    return true;
  }
  return false;  
}

boolean WM8960::setSMD(boolean mode)
{
  return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 5, mode);
}

 // 0=integer, 1=fractional
boolean WM8960::setCLKSEL(boolean sel)
{
  return WM8960::_writeRegisterBit(WM8960_REG_CLOCKING_1, 0, sel);
}

 // 0=MCLK, 1=PLL_output
boolean WM8960::setSYSCLKDIV(uint8_t div) // (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,2,1,div);  
}

boolean WM8960::setADCDIV(uint8_t div) // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,8,6,div);  
}

boolean WM8960::setDACDIV(uint8_t div) // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,5,3,div);  
}

boolean WM8960::setBCLKDIV(uint8_t div)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,3,0,div);  
}

boolean WM8960::setDCLKDIV(uint8_t div) // Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,8,6,div);
}

boolean WM8960::setALRCGPIO()
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

boolean WM8960::setWL(uint8_t word_length)
{
  return WM8960::_writeRegisterMultiBits(WM8960_REG_AUDIO_INTERFACE_1,3,2,word_length);  
}