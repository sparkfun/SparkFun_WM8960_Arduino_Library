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

#ifndef __SparkFunWM8960_H__
#define __SparkFunWM8960_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

// I2C address (7-bit format for Wire library)
#define WM8960_ADDR 0x34

// WM8960 registers
#define WM8960_REG_LEFT_INPUT_VOLUME 0x00
#define WM8960_REG_RIGHT_INPUT_VOLUME 0x01
#define WM8960_REG_LOUT1_VOLUME 0x02
#define WM8960_REG_ROUT1_VOLUME 0x03
#define WM8960_REG_CLOCKING_1 0x04
#define WM8960_REG_ADC_DAC_CTRL_1 0x05
#define WM8960_REG_ADC_DAC_CTRL_2 0x06
#define WM8960_REG_AUDIO_INTERFACE_1 0x07
#define WM8960_REG_CLOCKING_2 0x08
#define WM8960_REG_AUDIO_INTERFACE_2 0x09
#define WM8960_REG_LEFT_DAC_VOLUME 0x0A
#define WM8960_REG_RIGHT_DAC_VOLUME 0x0B
#define WM8960_REG_RESET 0x0F
#define WM8960_REG_3D_CONTROL 0x10
#define WM8960_REG_ALC1 0x11
#define WM8960_REG_ALC2 0x12
#define WM8960_REG_ALC3 0x13
#define WM8960_REG_NOISE_GATE 0x14
#define WM8960_REG_LEFT_ADC_VOLUME 0x15
#define WM8960_REG_RIGHT_ADC_VOLUME 0x16
#define WM8960_REG_ADDITIONAL_CONTROL_1 0x17
#define WM8960_REG_ADDITIONAL_CONTROL_2 0x18
#define WM8960_REG_PWR_MGMT_1 0x19
#define WM8960_REG_PWR_MGMT_2 0x1A
#define WM8960_REG_ADDITIONAL_CONTROL_3 0x1B
#define WM8960_REG_ANTI_POP_1 0x1C
#define WM8960_REG_ANTI_POP_2 0x1D
#define WM8960_REG_ADCL_SIGNAL_PATH 0x20
#define WM8960_REG_ADCR_SIGNAL_PATH 0x21
#define WM8960_REG_LEFT_OUT_MIX_1 0x22
#define WM8960_REG_RIGHT_OUT_MIX_2 0x25
#define WM8960_REG_MONO_OUT_MIX_1 0x26
#define WM8960_REG_MONO_OUT_MIX_2 0x27
#define WM8960_REG_LOUT2_VOLUME 0x28
#define WM8960_REG_ROUT2_VOLUME 0x29
#define WM8960_REG_MONO_OUT_VOLUME 0x2A
#define WM8960_REG_INPUT_BOOST_MIXER_1 0x2B
#define WM8960_REG_INPUT_BOOST_MIXER_2 0x2C
#define WM8960_REG_BYPASS_1 0x2D
#define WM8960_REG_BYPASS_2 0x2E
#define WM8960_REG_PWR_MGMT_3 0x2F
#define WM8960_REG_ADDITIONAL_CONTROL_4 0x30
#define WM8960_REG_CLASS_D_CONTROL_1 0x31
#define WM8960_REG_CLASS_D_CONTROL_3 0x33
#define WM8960_REG_PLL_N 0x34
#define WM8960_REG_PLL_K_1 0x35
#define WM8960_REG_PLL_K_2 0x36
#define WM8960_REG_PLL_K_3 0x37

// PGA input selections
#define PGAL_LINPUT2 0
#define PGAL_LINPUT3 1
#define PGAL_VMID 2
#define PGAR_RINPUT2 0
#define PGAR_RINPUT3 1
#define PGAR_VMID 2

// Mic (aka PGA) BOOST gain options
#define MIC_BOOST_GAIN_0DB 0
#define MIC_BOOST_GAIN_13DB 1
#define MIC_BOOST_GAIN_20DB 2
#define MIC_BOOST_GAIN_29DB 3

// Mixer 1 and 2 boost gain options
#define MIXER_BOOST_GAIN_MUTE 0
#define MIXER_BOOST_GAIN_NEG_12DB 1
#define MIXER_BOOST_GAIN_NEG_9DB 2
#define MIXER_BOOST_GAIN_NEG_6DB 3
#define MIXER_BOOST_GAIN_NEG_3DB 4
#define MIXER_BOOST_GAIN_0DB 5
#define MIXER_BOOST_GAIN_3DB 6
#define MIXER_BOOST_GAIN_6DB 7

// Mic Bias voltage options
#define MIC_BIAS_VOLTAGE_0_9_AVDD 0
#define MIC_BIAS_VOLTAGE_0_65_AVDD 1

// Automatic Level Control Modes
#define ALC_MODE_OFF 0
#define ALC_MODE_RIGHT_ONLY 1
#define ALC_MODE_LEFT_ONLY 2
#define ALC_MODE_STEREO 3

class WM8960
{
	public:
		WM8960();
		boolean begin(TwoWire &wirePort = Wire);
		boolean isConnected();

		boolean enableVREF(); // necessary for all other functions
		boolean disableVREF(); // use for turning this off to save power
		
		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// PGA 
		/////////////////////////////////////////////////////////

		boolean pgaLeftEnable();
		boolean pgaLeftDisable();
		boolean pgaRightEnable();
		boolean pgaRightDisable();

		boolean enable_LMICBOOST();
		boolean disable_LMICBOOST();
		boolean enable_RMICBOOST();
		boolean disable_RMICBOOST();

		// PGA input signal select
		// Each PGA (left and right) has a switch on its non-inverting input.
		// On PGA_LEFT:
		// 	*You can select between VMID, LINPUT2 or LINPUT3
		// 	*Note, the inverting input of PGA_LEFT is perminantly connected to LINPUT1
		// On PGA_RIGHT:
		//	*You can select between VMIN, RINPUT2 or RINPUT3
		// 	*Note, the inverting input of PGA_RIGHT is perminantly connected to RINPUT1

		boolean pgaLeftNonInvSignalSelect(char signal); // 3 options: PGAL_LINPUT2, PGAL_LINPUT3, PGAL_VMID
		boolean pgaRightNonInvSignalSelect(char signal); // 3 options: PGAR_RINPUT2, PGAR_RINPUT3, PGAR_VMID

		// Connection from each INPUT1 to the inverting input of its PGA
		boolean pgaLeftInvSignalConnect(); 		// Connect LINPUT1 to inverting input of Left Input PGA
		boolean pgaLeftInvSignalDisconnect(); 	// Disconnect LINPUT1 from inverting input of Left Input PGA
		boolean pgaRightInvSignalConnect(); 	// Connect RINPUT1 to inverting input of Right Input PGA
		boolean pgaRightInvSignalDisconnect(); 	// Disconnect RINPUT1 from inverting input of Right Input PGA	

		// Connection from output of PGAs to downstream "boost mixers".
		boolean connect_LMIC2B(); 		// Connect Left Input PGA to Left Input Boost mixer
		boolean disconnect_LMIC2B();	// Disconnect Left Input PGA to Left Input Boost mixer
		boolean connect_RMIC2B(); 		// Connect Right Input PGA to Right Input Boost mixer
		boolean disconnect_RMIC2B();	// Disconnect Right Input PGA to Right Input Boost mixer

		boolean pgaLeftSetVolume(uint8_t volume); // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
		boolean pgaRightSetVolume(uint8_t volume); // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)

		// Zero Cross prevents zipper sounds on volume changes
		boolean pgaZeroCrossOn(); // sets both left and right PGAs
		boolean pgaZeroCrossOff(); // sets both left and right PGAs

		boolean pgaLeftMuteOn();
		boolean pgaLeftMuteOff();
		boolean pgaRightMuteOn();
		boolean pgaRightMuteOff();

		boolean pgaLeftIPVUSet(); // causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
		boolean pgaRightIPVUSet(); // causes left and right input PGA volumes to be updated (LINVOL and RINVOL)

		// Boosts
		boolean set_LMICBOOST(uint8_t boost_gain); // MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
		boolean set_RMICBOOST(uint8_t boost_gain); // MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
		boolean set_LIN3BOOST(uint8_t boost_gain); // MIXER_BOOST_GAIN_MUTE, MIXER_BOOST_GAIN_NEG_12DB, and so on...
		boolean set_LIN2BOOST(uint8_t boost_gain); // MIXER_BOOST_GAIN_MUTE, MIXER_BOOST_GAIN_NEG_12DB, and so on...
		boolean set_RIN3BOOST(uint8_t boost_gain); // MIXER_BOOST_GAIN_MUTE, MIXER_BOOST_GAIN_NEG_12DB, and so on...
		boolean set_RIN2BOOST(uint8_t boost_gain); // MIXER_BOOST_GAIN_MUTE, MIXER_BOOST_GAIN_NEG_12DB, and so on...		

		// Mic Bias control
		boolean enableMicBias();
		boolean disableMicBias();
		boolean setMicBiasVoltage(boolean voltage); // MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ADC
		/////////////////////////////////////////////////////////

		boolean enableAdcLeft();
		boolean disableAdcLeft();
		boolean enableAdcRight();
		boolean disableAdcRight();

		// ADC digital volume
		// note, also needs to handle control of the ADCVU bits (volume update).
		// valid inputs are 0-255
		// 0 = mute
		// 1 = -97dB
		// ... 0.5dB steps up to
		// 255 = +30dB
		boolean setAdcLeftDigitalVolume(uint8_t volume); 
		boolean setAdcRightDigitalVolume(uint8_t volume);

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ALC
		/////////////////////////////////////////////////////////

		// Automatic Level Control
		// Note that when the ALC function is enabled, the settings of
		// registers 0 and 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and RINMUTE) are ignored.
		boolean enableAlc(uint8_t mode = ALC_MODE_STEREO); // also sets alc sample rate to match global sample rate.
		boolean disableAlc();

		boolean setAlcTarget(uint8_t target); // valid inputs are 0-15
		boolean setAlcDecay(uint8_t decay); // valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
		boolean setAlcAttack(uint8_t attack); // valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds

		// Peak Limiter
		enablePeakLimiter();
		disablePeakLimiter();

		// Noise Gate
		boolean enableNoiseGate();
		boolean disableNoiseGate();
		boolean setNoiseGateThreshold(uint8_t threshold); // 0-31, 0 = -76.5dBfs, 31 = -30dBfs

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// DAC
		/////////////////////////////////////////////////////////

		// enable/disble each channel
		boolean enableDacLeft();
		boolean disableDacLeft();
		boolean enableDacRight();
		boolean disableDacRight();

		// DAC digital volume
		// note, also needs to handle control of the DACVU bits (volume update).
		// valid inputs are 0-255
		// 0 = mute
		// 1 = -127dB
		// ... 0.5dB steps up to
		// 255 = 0dB
		boolean setDacLeftDigitalVolume(uint8_t volume); 
		boolean setDacRightDigitalVolume(uint8_t volume);		


		// DAC mute
		enableDacMute();
		disableDacMute();

		// DE-Emphasis

		// 3D Stereo Enhancement
		// 3D enable/disable
		boolean enable3d();
		boolean disable3d();
		boolean set3dDepth(uint8_t depth); // 0 = 0%, 15 = 100%

		// 3D upper/lower cut-off frequencies.

		// DAC output -6dB attentuation enable/disable
		boolean enableDac6dbAttenuation();
		boolean disableDac6dbAttentuation();

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// OUTPUT mixers
		/////////////////////////////////////////////////////////

		// what's connected to what? Oh so many options...
		// LOMIX	Left Output Mixer
		// ROMIX	Right Output Mixer
		// OUT3MIX		Mono Output Mixer

		// enable/disable left and right output mixers
		boolean enableLOMIX();
		boolean disableLOMIX();
		boolean enableROMIX();
		boolean disableROMIX();
		boolean enableOUT3MIX();
		boolean disableOUT3MIX();

		// enable/disable audio path connections/vols to/from output mixers
		// see datasheet page 35 for a nice image of all the connections.

		boolean enableLI2LO();
		boolean disableLI2LO();
		boolean setLI2LOVOL(uint8_t volume); // 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB

		boolean enableLB2LO();
		boolean disableLB2LO();
		boolean setLB2LOVOL(uint8_t volume); // 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB

		boolean enableLD2LO();
		boolean disableLD2LO();

		boolean enableRI2RO();
		boolean disableRI2RO();
		boolean setRI2ROVOL(uint8_t volume); // 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB

		boolean enableRB2RO();
		boolean disableRB2RO();
		boolean setRB2ROVOL(uint8_t volume); // 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB

		boolean enableRD2RO();
		boolean disableRD2RO();

		// Mono Output mixer. 
		// note, for capless HPs, we'll want this to output a buffered VMID.
		// to do this, we need to disable both of these connections.
		boolean enableLI2MO();
		boolean disableLI2MO();
		boolean enableRI2MO();
		boolean disableRI2MO();
		boolean enableOUT3asVMID(); // this will disable both connections, thus enable VMID on OUT3
		// note, to enable VMID, you also need to enable OUT3 in the WM8960_REG_PWR_MGMT_2 [1]
		boolean enableVMID(); // enables VMID in the WM8960_REG_PWR_MGMT_1 register, and set's it to playback/record settings of 2*50Kohm.
		boolean disableVMID();

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Headphones
		/////////////////////////////////////////////////////////

		// Enable and disable headphones (mute)
		unsigned char enableHeadphones();
		unsigned char disableHeadphones();
		unsigned char enableRightHeadphone();
		unsigned char disableRightHeadphone();
		unsigned char enableLeftHeadphone();
		unsigned char disableLeftHeadphone();

		boolean enableHeadphoneStandby();
		boolean disableHeadphoneStandby();

		// Although you can control each headphone output independently, here we are
		// going to assume you want both left and right to do the same thing.
		// Set volume
		boolean setHeadphoneVolume(uint8_t volume); // Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
		// updates both left and right channels
		// handles the OUT1VU (volume update) bit control, so that it happens at the same time on both channels.
		// Note, we must also make sure that the outputs are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]

		// Zero Cross prevents zipper sounds on volume changes
		boolean headphoneZeroCrossOn(); // sets both left and right Headphone outputs
		boolean headphoneZeroCrossOff(); // sets both left and right Headphone outputs
		

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Speakers
		/////////////////////////////////////////////////////////

		// Enable and disable speakers (mute)
		unsigned char enableSpeakers();
		unsigned char disableSpeakers();
		unsigned char enableRightSpeaker();
		unsigned char disableRightSpeaker();
		unsigned char enableLeftSpeaker();
		unsigned char disableLeftSpeaker();

		// Although you can control each Speaker output independently, here we are
		// going to assume you want both left and right to do the same thing.
		// Set volume
		boolean setSpeakerVolume(uint8_t volume); // Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
		// updates both left and right channels
		// handles the SPKVU (volume update) bit control, so that it happens at the same time on both channels.
		// Note, we must also make sure that the outputs are enabled in the WM8960_REG_PWR_MGMT_2 [4:3]
		// and the class D control reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

		// Zero Cross prevents zipper sounds on volume changes
		boolean speakerZeroCrossOn(); // sets both left and right Speaker outputs
		boolean speakerZeroCrossOff(); // sets both left and right Speaker outputs		

		// DC and AC gain - allows signal to be higher than the DACs swing
		// (use only if your SPKVDD is high enough to handle a larger signal)
		// valid inputs are 0-5
		// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
		boolean setSpeakerDcGain(uint8_t gain);
		boolean setSpeakerAcGain(uint8_t gain);

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Digital audio interface control
		/////////////////////////////////////////////////////////

		// defaults to I2S, peripheral-mode, 24-bit word length
		// *Might need to change the WL to match our application. 
		// *I believe the BT example will need 16-bit WL.
		boolean setAudioDataWordLength(uint8_t length); // 0=16bit, 1=20bit, 2=24bit, 3=32bit.

		// Loopback
		// When enabled, the output data from the ADC audio interface is fed directly into the DAC data input.
		boolean enableLoopBack();
		boolean disableLoopBack();

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
		boolean enablePLL();
		boolean disablePLL();
		boolean set_PLLPRESCALE(boolean val = 1); // (0=divide by 1), (1=div by 2)
		boolean set_PLLN(uint8_t n);
		boolean set_PLLK(uint8_t one, uint8_t two, uint8_t three); // send each nibble of 24-bit value for value K
		boolean set_SMD(boolean mode); // 0=integer, 1=fractional
		boolean set_CLKSEL(boolean sel); // 0=MCLK, 1=PLL_output
		boolean set_SYSCLKDIV(uint8_t div = 2); // (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
		boolean set_ADVDIV(uint8_t setting); // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
		boolean set_DACDIV(uint8_t setting); // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
		boolean set_DCLKDIV(uint8_t setting); // Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz

		// General-purpose register read/write
		unsigned char writeRegister(unsigned char reg, unsigned char value);
		unsigned char readRegister(unsigned char reg, unsigned char *value);

	private:
		TwoWire *_i2cPort;
		uint8_t _deviceAddress = WM8960_ADDR;
};
#endif
