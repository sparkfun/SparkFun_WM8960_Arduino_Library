/******************************************************************************
  SparkFun WM8960 Arduino Library

  This library provides a set of functions to control (via I2C) the Wolfson 
  Microelectronics WM8960 Stereo CODEC with 1W Stereo Class D Speaker Drivers 
  and Headphone Drivers.

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

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions 
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SPARKFUN_WM8960_H__
#define __SPARKFUN_WM8960_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

// I2C address (7-bit format for Wire library)
#define WM8960_ADDR 0x1A 

// WM8960 register addresses
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
#define WM8960_PGAL_LINPUT2 0
#define WM8960_PGAL_LINPUT3 1
#define WM8960_PGAL_VMID 2
#define WM8960_PGAR_RINPUT2 0
#define WM8960_PGAR_RINPUT3 1
#define WM8960_PGAR_VMID 2

// Mic (aka PGA) BOOST gain options
#define WM8960_MIC_BOOST_GAIN_0DB 0
#define WM8960_MIC_BOOST_GAIN_13DB 1
#define WM8960_MIC_BOOST_GAIN_20DB 2
#define WM8960_MIC_BOOST_GAIN_29DB 3

// Boost Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LIN2BOOST
// LIN3BOOST
// RIN2BOOST
// RIN3BOOST
#define WM8960_BOOST_MIXER_GAIN_MUTE 0
#define WM8960_BOOST_MIXER_GAIN_NEG_12DB 1
#define WM8960_BOOST_MIXER_GAIN_NEG_9DB 2
#define WM8960_BOOST_MIXER_GAIN_NEG_6DB 3
#define WM8960_BOOST_MIXER_GAIN_NEG_3DB 4
#define WM8960_BOOST_MIXER_GAIN_0DB 5
#define WM8960_BOOST_MIXER_GAIN_3DB 6
#define WM8960_BOOST_MIXER_GAIN_6DB 7

// Output Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LI2LOVOL
// LB2LOVOL
// RI2LOVOL
// RB2LOVOL
// These are useful as analog bypass signal path options.
#define WM8960_OUTPUT_MIXER_GAIN_0DB 0
#define WM8960_OUTPUT_MIXER_GAIN_NEG_3DB 1
#define WM8960_OUTPUT_MIXER_GAIN_NEG_6DB 2
#define WM8960_OUTPUT_MIXER_GAIN_NEG_9DB 3
#define WM8960_OUTPUT_MIXER_GAIN_NEG_12DB 4
#define WM8960_OUTPUT_MIXER_GAIN_NEG_15DB 5
#define WM8960_OUTPUT_MIXER_GAIN_NEG_18DB 6
#define WM8960_OUTPUT_MIXER_GAIN_NEG_21DB 7

// Mic Bias voltage options
#define WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD 0
#define WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD 1

// SYSCLK divide
#define WM8960_SYSCLK_DIV_BY_1 0
#define WM8960_SYSCLK_DIV_BY_2 2
#define WM8960_CLKSEL_MCLK 0
#define WM8960_CLKSEL_PLL 1
#define WM8960_PLL_MODE_INTEGER 0
#define WM8960_PLL_MODE_FRACTIONAL 1
#define WM8960_PLLPRESCALE_DIV_1 0
#define WM8960_PLLPRESCALE_DIV_2 1

// Class d clock divide
#define WM8960_DCLKDIV_16 7

// Word length settings (aka bits per sample)
// Audio Data Word Length
#define WM8960_WL_16BIT 0
#define WM8960_WL_20BIT 1
#define WM8960_WL_24BIT 2
#define WM8960_WL_32BIT 3

// Additional Digital Audio Interface controls
// LRP (aka left-right-polarity)
// Right, left and I2S modes – LRCLK polarity 
// 0 = normal LRCLK polarity 
// 1 = inverted LRCLK polarity 
#define WM8960_LR_POLARITY_NORMAL 0
#define WM8960_LR_POLARITY_INVERT 1

// ALRSWAP (aka ADC left/right swap)
// Left/Right ADC channel swap
// 1 = Swap left and right ADC data in audio interface
// 0 = Output left and right data as normal 
#define WM8960_ALRSWAP_NORMAL 0
#define WM8960_ALRSWAP_SWAP 1

// Gain mins, maxes, offsets and step-sizes for all the amps within the codec.
#define WM8960_PGA_GAIN_MIN -17.25
#define WM8960_PGA_GAIN_MAX 30.00
#define WM8960_PGA_GAIN_OFFSET 17.25
#define WM8960_PGA_GAIN_STEPSIZE 0.75
#define WM8960_HP_GAIN_MIN -73.00
#define WM8960_HP_GAIN_MAX 6.00
#define WM8960_HP_GAIN_OFFSET 121.00
#define WM8960_HP_GAIN_STEPSIZE 1.00
#define WM8960_SPEAKER_GAIN_MIN -73.00
#define WM8960_SPEAKER_GAIN_MAX 6.00
#define WM8960_SPEAKER_GAIN_OFFSET 121.00
#define WM8960_SPEAKER_GAIN_STEPSIZE 1.00
#define WM8960_ADC_GAIN_MIN -97.00
#define WM8960_ADC_GAIN_MAX 30.00
#define WM8960_ADC_GAIN_OFFSET 97.50
#define WM8960_ADC_GAIN_STEPSIZE 0.50
#define WM8960_DAC_GAIN_MIN -97.00
#define WM8960_DAC_GAIN_MAX 30.00
#define WM8960_DAC_GAIN_OFFSET 97.50
#define WM8960_DAC_GAIN_STEPSIZE 0.50

// Automatic Level Control Modes
#define WM8960_ALC_MODE_OFF 0
#define WM8960_ALC_MODE_RIGHT_ONLY 1
#define WM8960_ALC_MODE_LEFT_ONLY 2
#define WM8960_ALC_MODE_STEREO 3

// Automatic Level Control Target Level dB
#define WM8960_ALC_TARGET_LEVEL_NEG_22_5DB 0
#define WM8960_ALC_TARGET_LEVEL_NEG_21DB 1
#define WM8960_ALC_TARGET_LEVEL_NEG_19_5DB 2
#define WM8960_ALC_TARGET_LEVEL_NEG_18DB 3
#define WM8960_ALC_TARGET_LEVEL_NEG_16_5DB 4
#define WM8960_ALC_TARGET_LEVEL_NEG_15DB 5
#define WM8960_ALC_TARGET_LEVEL_NEG_13_5DB 6
#define WM8960_ALC_TARGET_LEVEL_NEG_12DB 7
#define WM8960_ALC_TARGET_LEVEL_NEG_10_5DB 8
#define WM8960_ALC_TARGET_LEVEL_NEG_9DB 9
#define WM8960_ALC_TARGET_LEVEL_NEG_7_5DB 10
#define WM8960_ALC_TARGET_LEVEL_NEG_6DB 11
#define WM8960_ALC_TARGET_LEVEL_NEG_4_5DB 12
#define WM8960_ALC_TARGET_LEVEL_NEG_3DB 13
#define WM8960_ALC_TARGET_LEVEL_NEG_1_5DB 14

// Automatic Level Control Max Gain Level dB
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_12DB 0
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_6DB 1
#define WM8960_ALC_MAX_GAIN_LEVEL_0DB 2
#define WM8960_ALC_MAX_GAIN_LEVEL_6DB 3
#define WM8960_ALC_MAX_GAIN_LEVEL_12DB 4
#define WM8960_ALC_MAX_GAIN_LEVEL_18DB 5
#define WM8960_ALC_MAX_GAIN_LEVEL_24DB 6
#define WM8960_ALC_MAX_GAIN_LEVEL_30DB 7

// Automatic Level Control Min Gain Level dB
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_17_25DB 0
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_11_25DB 1
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_5_25DB 2
#define WM8960_ALC_MIN_GAIN_LEVEL_0_75DB 3
#define WM8960_ALC_MIN_GAIN_LEVEL_6_75DB 4
#define WM8960_ALC_MIN_GAIN_LEVEL_12_75DB 5
#define WM8960_ALC_MIN_GAIN_LEVEL_18_75DB 6
#define WM8960_ALC_MIN_GAIN_LEVEL_24_75DB 7

// Automatic Level Control Hold Time (MS and SEC)
#define WM8960_ALC_HOLD_TIME_0MS 0
#define WM8960_ALC_HOLD_TIME_3MS 1
#define WM8960_ALC_HOLD_TIME_5MS 2
#define WM8960_ALC_HOLD_TIME_11MS 3
#define WM8960_ALC_HOLD_TIME_21MS 4
#define WM8960_ALC_HOLD_TIME_43MS 5
#define WM8960_ALC_HOLD_TIME_85MS 6
#define WM8960_ALC_HOLD_TIME_170MS 7
#define WM8960_ALC_HOLD_TIME_341MS 8
#define WM8960_ALC_HOLD_TIME_682MS 9
#define WM8960_ALC_HOLD_TIME_1365MS 10
#define WM8960_ALC_HOLD_TIME_3SEC 11
#define WM8960_ALC_HOLD_TIME_5SEC 12
#define WM8960_ALC_HOLD_TIME_10SEC 13
#define WM8960_ALC_HOLD_TIME_23SEC 14
#define WM8960_ALC_HOLD_TIME_44SEC 15

// Automatic Level Control Decay Time (MS and SEC)
#define WM8960_ALC_DECAY_TIME_24MS 0
#define WM8960_ALC_DECAY_TIME_48MS 1
#define WM8960_ALC_DECAY_TIME_96MS 2
#define WM8960_ALC_DECAY_TIME_192MS 3
#define WM8960_ALC_DECAY_TIME_384MS 4
#define WM8960_ALC_DECAY_TIME_768MS 5
#define WM8960_ALC_DECAY_TIME_1536MS 6
#define WM8960_ALC_DECAY_TIME_3SEC 7
#define WM8960_ALC_DECAY_TIME_6SEC 8
#define WM8960_ALC_DECAY_TIME_12SEC 9
#define WM8960_ALC_DECAY_TIME_24SEC 10

// Automatic Level Control Attack Time (MS and SEC)
#define WM8960_ALC_ATTACK_TIME_6MS 0
#define WM8960_ALC_ATTACK_TIME_12MS 1
#define WM8960_ALC_ATTACK_TIME_24MS 2
#define WM8960_ALC_ATTACK_TIME_482MS 3
#define WM8960_ALC_ATTACK_TIME_964MS 4
#define WM8960_ALC_ATTACK_TIME_1928MS 5
#define WM8960_ALC_ATTACK_TIME_3846MS 6
#define WM8960_ALC_ATTACK_TIME_768MS 7
#define WM8960_ALC_ATTACK_TIME_1536MS 8
#define WM8960_ALC_ATTACK_TIME_3SEC 9
#define WM8960_ALC_ATTACK_TIME_6SEC 10

// Speaker Boost Gains (DC and AC)
#define WM8960_SPEAKER_BOOST_GAIN_0DB 0
#define WM8960_SPEAKER_BOOST_GAIN_2_1DB 1
#define WM8960_SPEAKER_BOOST_GAIN_2_9DB 2
#define WM8960_SPEAKER_BOOST_GAIN_3_6DB 3
#define WM8960_SPEAKER_BOOST_GAIN_4_5DB 4
#define WM8960_SPEAKER_BOOST_GAIN_5_1DB 5


class WM8960
{
	public:
		WM8960();
		boolean begin(TwoWire &wirePort = Wire);
		boolean isConnected();

		boolean enableVREF(); // Necessary for all other functions
		boolean disableVREF(); // Use for turning this off to save power

		boolean reset(); // Resets all registers to their default state

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// PGA 
		/////////////////////////////////////////////////////////

		boolean enableAINL();
		boolean disableAINL();
		boolean enableAINR();
		boolean disableAINR();

		boolean enableLMIC();
		boolean disableLMIC();
		boolean enableRMIC();
		boolean disableRMIC();

		boolean enableLMICBOOST();
		boolean disableLMICBOOST();
		boolean enableRMICBOOST();
		boolean disableRMICBOOST();

		// PGA input signal select
		// Each PGA (left and right) has a switch on its non-inverting input.
		// On PGA_LEFT:
		// You can select between VMID, LINPUT2 or LINPUT3
		// Note, the inverting input of PGA_LEFT is perminantly connected to 
		// LINPUT1
		// On PGA_RIGHT:
		// You can select between VMIN, RINPUT2 or RINPUT3
		// Note, the inverting input of PGA_RIGHT is perminantly connected to 
		// RINPUT1

		// 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
		boolean pgaLeftNonInvSignalSelect(uint8_t signal); 

		// 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
		boolean pgaRightNonInvSignalSelect(uint8_t signal); 

		// Connections from each INPUT1 to the inverting input of its PGA
	
		// Connect LINPUT1 to inverting input of Left Input PGA
		boolean connectLMN1(); 
	
		// Disconnect LINPUT1 from inverting input of Left Input PGA
		boolean disconnectLMN1(); 	

		// Connect RINPUT1 to inverting input of Right Input PGA
		boolean connectRMN1(); 	

		// Disconnect RINPUT1 from inverting input of Right Input PGA
		boolean disconnectRMN1(); 		

		// Connection from output of PGAs to downstream "boost mixers"

		// Connect Left Input PGA to Left Input Boost mixer
		boolean connectLMIC2B(); 		

		// Disconnect Left Input PGA to Left Input Boost mixer
		boolean disconnectLMIC2B();	

		// Connect Right Input PGA to Right Input Boost mixer
		boolean connectRMIC2B(); 	

		// Disconnect Right Input PGA to Right Input Boost mixer	
		boolean disconnectRMIC2B();	

		// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
		boolean setLINVOL(uint8_t volume); 
		boolean setLINVOLDB(float dB);

		// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
		boolean setRINVOL(uint8_t volume); 
		boolean setRINVOLDB(float dB);

		// Zero Cross prevents zipper sounds on volume changes
		boolean enablePgaZeroCross(); // Sets both left and right PGAs
		boolean disablePgaZeroCross(); // Sets both left and right PGAs

		boolean enableLINMUTE();
		boolean disableLINMUTE();
		boolean enableRINMUTE();
		boolean disableRINMUTE();

		// Causes left and right input PGA volumes to be updated 
		// (LINVOL and RINVOL)
		boolean pgaLeftIPVUSet(); 

		// Causes left and right input PGA volumes to be updated 
		// (LINVOL and RINVOL)
		boolean pgaRightIPVUSet(); 

		// Boosts

		// WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
		boolean setLMICBOOST(uint8_t boost_gain); 

		// WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
		boolean setRMICBOOST(uint8_t boost_gain); 

		// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
		boolean setLIN3BOOST(uint8_t boost_gain); 

		// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
		boolean setLIN2BOOST(uint8_t boost_gain); 

		// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
		boolean setRIN3BOOST(uint8_t boost_gain); 

		// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
		boolean setRIN2BOOST(uint8_t boost_gain); 

		// Mic Bias control
		boolean enableMicBias();
		boolean disableMicBias();

		// WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) 
		// or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
		boolean setMicBiasVoltage(boolean voltage); 

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ADC
		/////////////////////////////////////////////////////////

		boolean enableAdcLeft();
		boolean disableAdcLeft();
		boolean enableAdcRight();
		boolean disableAdcRight();

		// ADC digital volume
		// Note, also needs to handle control of the ADCVU bits (volume update).
		// Valid inputs are 0-255
		// 0 = mute
		// 1 = -97dB
		// ... 0.5dB steps up to
		// 195 = 0dB
		// 255 = +30dB
		boolean setAdcLeftDigitalVolume(uint8_t volume); 
		boolean setAdcRightDigitalVolume(uint8_t volume);
		boolean setAdcLeftDigitalVolumeDB(float dB); 
		boolean setAdcRightDigitalVolumeDB(float dB);

		// Causes left and right input ADC volumes to be updated
		boolean adcLeftADCVUSet(); 

		// Causes left and right input ADC volumes to be updated
		boolean adcRightADCVUSet(); 

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// ALC
		/////////////////////////////////////////////////////////

		// Automatic Level Control
		// Note that when the ALC function is enabled, the settings of
		// Registers 0 and 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and 
		// RINMUTE) are ignored.

		// Also sets alc sample rate to match global sample rate.
		boolean enableAlc(uint8_t mode = WM8960_ALC_MODE_STEREO); 
		boolean disableAlc();

		// Valid inputs are 0-15
		// 0 = -22.5dB FS ... 1.5dB steps ... 15 = -1.5dB FS
		boolean setAlcTarget(uint8_t target); 

		// Valid inputs are 0-10, 0 = 24ms, 1 = 48ms ... 10 = 24.58seconds
		boolean setAlcDecay(uint8_t decay); 

		// Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms ... 
		// 10 = 6.14seconds
		boolean setAlcAttack(uint8_t attack); 

		// Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
		boolean setAlcMaxGain(uint8_t maxGain);

		// Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB 
		boolean setAlcMinGain(uint8_t attack); 

		// Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
		boolean setAlcHold(uint8_t attack); 

		// Peak Limiter
		boolean enablePeakLimiter();
		boolean disablePeakLimiter();

		// Noise Gate
		boolean enableNoiseGate();
		boolean disableNoiseGate();

		// 0-31, 0 = -76.5dBfs, 31 = -30dBfs
		boolean setNoiseGateThreshold(uint8_t threshold); 

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// DAC
		/////////////////////////////////////////////////////////

		// Enable/disble each channel
		boolean enableDacLeft();
		boolean disableDacLeft();
		boolean enableDacRight();
		boolean disableDacRight();

		// DAC digital volume
		// Note, also needs to handle control of the DACVU bits (volume update).
		// Valid inputs are 0-255
		// 0 = mute
		// 1 = -127dB
		// ... 0.5dB steps up to
		// 255 = 0dB
		boolean setDacLeftDigitalVolume(uint8_t volume); 
		boolean setDacRightDigitalVolume(uint8_t volume);	
		boolean setDacLeftDigitalVolumeDB(float dB); 
		boolean setDacRightDigitalVolumeDB(float dB);	

		// Causes left and right input DAC volumes to be updated
		boolean dacLeftDACVUSet(); 

		// Causes left and right input DAC volumes to be updated
		boolean dacRightDACVUSet(); 

		// DAC mute
		boolean enableDacMute();
		boolean disableDacMute();

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

		//////////////////////////////////////////////////////
		////////////////////////////////////////////////////// OUTPUT mixers
		//////////////////////////////////////////////////////

		// What's connected to what? Oh so many options...
		// LOMIX	Left Output Mixer
		// ROMIX	Right Output Mixer
		// OUT3MIX		Mono Output Mixer

		// Enable/disable left and right output mixers
		boolean enableLOMIX();
		boolean disableLOMIX();
		boolean enableROMIX();
		boolean disableROMIX();
		boolean enableOUT3MIX();
		boolean disableOUT3MIX();

		// Enable/disable audio path connections/vols to/from output mixers
		// See datasheet page 35 for a nice image of all the connections.

		boolean enableLI2LO();
		boolean disableLI2LO();

		// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
		boolean setLI2LOVOL(uint8_t volume); 

		boolean enableLB2LO();
		boolean disableLB2LO();

		// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
		boolean setLB2LOVOL(uint8_t volume); 

		boolean enableLD2LO();
		boolean disableLD2LO();

		boolean enableRI2RO();
		boolean disableRI2RO();

		// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
		boolean setRI2ROVOL(uint8_t volume); 

		boolean enableRB2RO();
		boolean disableRB2RO();

		// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
		boolean setRB2ROVOL(uint8_t volume); 

		boolean enableRD2RO();
		boolean disableRD2RO();

		// Mono Output mixer. 
		// Note, for capless HPs, we'll want this to output a buffered VMID.
		// To do this, we need to disable both of these connections.
		boolean enableLI2MO();
		boolean disableLI2MO();
		boolean enableRI2MO();
		boolean disableRI2MO();

		// This will disable both connections, thus enable VMID on OUT3. Note, 
		// to enable VMID, you also need to enable OUT3 in the 
		// WM8960_REG_PWR_MGMT_2 [1]
		boolean enableOUT3asVMID(); 

		// Enables VMID in the WM8960_REG_PWR_MGMT_1 register, and set's it to 
		// playback/record settings of 2*50Kohm.
		boolean enableVMID(); 
		boolean disableVMID();

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Headphones
		/////////////////////////////////////////////////////////

		// Enable and disable headphones (mute)
		boolean enableHeadphones();
		boolean disableHeadphones();
		boolean enableRightHeadphone();
		boolean disableRightHeadphone();
		boolean enableLeftHeadphone();
		boolean disableLeftHeadphone();

		boolean enableHeadphoneStandby();
		boolean disableHeadphoneStandby();

		// Set headphone volume
		// Although you can control each headphone output independently, here 
		// we are going to assume you want both left and right to do the same 
		// thing.

		// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ...
		// 127 = +6dB
		boolean setHeadphoneVolume(uint8_t volume); 
		// Updates both left and right channels
		// Handles the OUT1VU (volume update) bit control, so that it happens at
		// the same time on both channels. Note, we must also make sure that the
		// outputs are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]

		// Zero Cross prevents zipper sounds on volume changes
		// Sets both left and right Headphone outputs
		boolean enableHeadphoneZeroCross(); 
		boolean disableHeadphoneZeroCross();

		// Set headphone volume dB
		// Sets the volume of the headphone output buffer amp to a speicified
		// dB value passed in as a float argument.
		// Valid dB settings are -74.0 up to +6.0
		// User input will be rounded to nearest whole integer
		// -74 (or lower) = MUTE
		// -73 = -73dB (MIN)
		// ... 1dB steps ...
		// 0 = 0dB
		// ... 1dB steps ...
		// 6 = +6dB  (MAX)
		boolean setHeadphoneVolumeDB(float dB);
		

		/////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Speakers
		/////////////////////////////////////////////////////////

		// Enable and disable speakers (mute)
		boolean enableSpeakers();
		boolean disableSpeakers();
		boolean enableRightSpeaker();
		boolean disableRightSpeaker();
		boolean enableLeftSpeaker();
		boolean disableLeftSpeaker();

		// Set Speaker output volume
		// Although you can control each Speaker output independently, here we 
		// are going to assume you want both left and right to do the same thing.
		// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ... 
		// 127 = +6dB

		boolean setSpeakerVolume(uint8_t volume); 
		// Updates both left and right channels
		// Handles the SPKVU (volume update) bit control, so that it happens at 
		// the same time on both channels. Note, we must also make sure that the 
		// outputs are enabled in the WM8960_REG_PWR_MGMT_2 [4:3]
		// And the class D control reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

		boolean setSpeakerVolumeDB(float dB);

		// Zero Cross prevents zipper sounds on volume changes
		// Sets both left and right Speaker outputs
		boolean enableSpeakerZeroCross(); 
		boolean disableSpeakerZeroCross();	

		// DC and AC gain - allows signal to be higher than the DACs swing
		// (use only if your SPKVDD is high enough to handle a larger signal)
		// Valid inputs are 0-5
		// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
		boolean setSpeakerDcGain(uint8_t gain);
		boolean setSpeakerAcGain(uint8_t gain);

		//////////////////////////////////////
		////////////////////////////////////// Digital audio interface control
		//////////////////////////////////////

		// Defaults to I2S, peripheral-mode, 24-bit word length

		// Loopback
		// When enabled, the output data from the ADC audio interface is fed 
		// directly into the DAC data input.
		boolean enableLoopBack();
		boolean disableLoopBack();

		///////////////////////////////////////////////////////
		/////////////////////////////////////////////////////// Clock controls
		///////////////////////////////////////////////////////

		// Getting the Frequency of SampleRate as we wish
		// Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
		// According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a 
		// SR of 44.1KHz. To get that Desired Output (SYSCLK), we need the 
		// following settings on the PLL stuff:
		// As found on table 45 (ds pg 61).
		// PRESCALE DIVIDE (PLLPRESCALE): 2
		// POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
		// FIXED POST-DIVIDE: 4
		// R: 7.5264 
		// N: 7h
		// K: 86C226h

		// Example at bottom of table 46, shows that we should be in fractional 
		// mode for a 44.1KHz.

		// In terms of registers, this is what we want for 44.1KHz
		// PLLEN=1			(PLL enable)
		// PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down 
		// to 12MHZ for F2
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

		// Valid options are WM8960_PLLPRESCALE_DIV_1, WM8960_PLLPRESCALE_DIV_2
		boolean setPLLPRESCALE(boolean div); 

		boolean setPLLN(uint8_t n);

		// Send each nibble of 24-bit value for value K
		boolean setPLLK(uint8_t one, uint8_t two, uint8_t three); 

		boolean setSMD(boolean mode); // 0=integer, 1=fractional
		boolean setCLKSEL(boolean sel); // 0=MCLK, 1=PLL_output

		// (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
		boolean setSYSCLKDIV(uint8_t div); 

		// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
		boolean setADCDIV(uint8_t div); 

		// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
		boolean setDACDIV(uint8_t div); 

		// 0100 (4) = sufficiently high for 24bit, div by 4 allows for max word 
		// length of 32bit
		boolean setBCLKDIV(uint8_t div); 

		// Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
		boolean setDCLKDIV(uint8_t div); 

		// Set LR clock to be the same for ADC & DAC (needed for loopback mode)
		boolean setALRCGPIO(); 

		boolean enableMasterMode();
		boolean enablePeripheralMode();

		boolean setWL(uint8_t word_length);

		boolean setLRP(boolean polarity);

		boolean setALRSWAP(boolean swap);

		// General-purpose register write
		boolean writeRegister(uint8_t reg, uint16_t value);

		// **The WM8960 does not support reading registers!!!

	private:
		TwoWire *_i2cPort;
		uint8_t _deviceAddress = WM8960_ADDR;
		boolean _writeRegisterBit(uint8_t registerAddress, uint8_t bitNumber, boolean bitValue);
		boolean _writeRegisterMultiBits(uint8_t registerAddress, uint8_t settingMsbNum, uint8_t settingLsbNum, uint8_t setting);
		uint8_t convertDBtoSetting(float dB, float offset, float stepSize, float minDB, float maxDB);

		// The WM8960 does not support I2C reads
		// This means we must keep a local copy of all the register values
		// We will instantiate with default values
		// As we write to the device, we will also make sure
		// To update our local copy as well, stored here in this array.
		// Each register is 9-bits, so we will store them as a uint16_t
		// They are in order from R0-R55, and we even keep blank spots for the
		// "reserved" registers. This way we can use the register address macro 
		// defines above to easiy access each local copy of each register.
		// Example: _registerLocalCopy[WM8960_REG_LEFT_INPUT_VOLUME]
		uint16_t _registerLocalCopy[56] = {
			0x0097, // R0 (0x00)
			0x0097, // R1 (0x01)
			0x0000, // R2 (0x02)
			0x0000, // R3 (0x03)
			0x0000, // R4 (0x04)
			0x0008, // F5 (0x05)
			0x0000, // R6 (0x06)
			0x000A, // R7 (0x07)
			0x01C0, // R8 (0x08)
			0x0000, // R9 (0x09)
			0x00FF, // R10 (0x0a)
			0x00FF, // R11 (0x0b)
			0x0000, // R12 (0x0C) RESERVED
			0x0000, // R13 (0x0D) RESERVED
			0x0000, // R14 (0x0E) RESERVED
			0x0000, // R15 (0x0F) RESERVED
			0x0000, // R16 (0x10)
			0x007B, // R17 (0x11)
			0x0100, // R18 (0x12)
			0x0032, // R19 (0x13)
			0x0000, // R20 (0x14)
			0x00C3, // R21 (0x15)
			0x00C3, // R22 (0x16)
			0x01C0, // R23 (0x17)
			0x0000, // R24 (0x18)
			0x0000, // R25 (0x19)
			0x0000, // R26 (0x1A)
			0x0000, // R27 (0x1B)
			0x0000, // R28 (0x1C)
			0x0000, // R29 (0x1D)
			0x0000, // R30 (0x1E) RESERVED
			0x0000, // R31 (0x1F) RESERVED
			0x0100, // R32 (0x20)
			0x0100, // R33 (0x21)
			0x0050, // R34 (0x22)
			0x0000, // R35 (0x23) RESERVED
			0x0000, // R36 (0x24) RESERVED
			0x0050, // R37 (0x25)
			0x0000, // R38 (0x26)
			0x0000, // R39 (0x27)
			0x0000, // R40 (0x28)
			0x0000, // R41 (0x29)
			0x0040, // R42 (0x2A)
			0x0000, // R43 (0x2B)
			0x0000, // R44 (0x2C)
			0x0050, // R45 (0x2D)
			0x0050, // R46 (0x2E)
			0x0000, // R47 (0x2F)
			0x0002, // R48 (0x30)
			0x0037, // R49 (0x31)
			0x0000, // R50 (0x32) RESERVED
			0x0080, // R51 (0x33)
			0x0008, // R52 (0x34)
			0x0031, // R53 (0x35)
			0x0026, // R54 (0x36)
			0x00e9, // R55 (0x37)
		};

		const uint16_t _registerDefaults[56] = {
			0x0097, // R0 (0x00)
			0x0097, // R1 (0x01)
			0x0000, // R2 (0x02)
			0x0000, // R3 (0x03)
			0x0000, // R4 (0x04)
			0x0008, // F5 (0x05)
			0x0000, // R6 (0x06)
			0x000A, // R7 (0x07)
			0x01C0, // R8 (0x08)
			0x0000, // R9 (0x09)
			0x00FF, // R10 (0x0a)
			0x00FF, // R11 (0x0b)
			0x0000, // R12 (0x0C) RESERVED
			0x0000, // R13 (0x0D) RESERVED
			0x0000, // R14 (0x0E) RESERVED
			0x0000, // R15 (0x0F) RESERVED
			0x0000, // R16 (0x10)
			0x007B, // R17 (0x11)
			0x0100, // R18 (0x12)
			0x0032, // R19 (0x13)
			0x0000, // R20 (0x14)
			0x00C3, // R21 (0x15)
			0x00C3, // R22 (0x16)
			0x01C0, // R23 (0x17)
			0x0000, // R24 (0x18)
			0x0000, // R25 (0x19)
			0x0000, // R26 (0x1A)
			0x0000, // R27 (0x1B)
			0x0000, // R28 (0x1C)
			0x0000, // R29 (0x1D)
			0x0000, // R30 (0x1E) RESERVED
			0x0000, // R31 (0x1F) RESERVED
			0x0100, // R32 (0x20)
			0x0100, // R33 (0x21)
			0x0050, // R34 (0x22)
			0x0000, // R35 (0x23) RESERVED
			0x0000, // R36 (0x24) RESERVED
			0x0050, // R37 (0x25)
			0x0000, // R38 (0x26)
			0x0000, // R39 (0x27)
			0x0000, // R40 (0x28)
			0x0000, // R41 (0x29)
			0x0040, // R42 (0x2A)
			0x0000, // R43 (0x2B)
			0x0000, // R44 (0x2C)
			0x0050, // R45 (0x2D)
			0x0050, // R46 (0x2E)
			0x0000, // R47 (0x2F)
			0x0002, // R48 (0x30)
			0x0037, // R49 (0x31)
			0x0000, // R50 (0x32) RESERVED
			0x0080, // R51 (0x33)
			0x0008, // R52 (0x34)
			0x0031, // R53 (0x35)
			0x0026, // R54 (0x36)
			0x00e9, // R55 (0x37)
		};		
};
#endif
