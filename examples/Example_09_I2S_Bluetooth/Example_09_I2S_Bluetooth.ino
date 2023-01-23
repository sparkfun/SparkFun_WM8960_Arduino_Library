/******************************************************************************
  Example_09_I2S_Bluetooth.ino
  Demonstrates how to receive audio via Bluetooth and play it back via I2S to the codec DAC.
  
  This example sets up the ESP32 as a bluetooth sink device, with its output set to I2S audio.
  
  This example sets up the WM8960 codec as an I2S peripheral, sets volume control, and Headphone output.

  A bluetooth device, such as your phone or laptop, can connect to the ESP32 and then begin playing an audio file.

  The ESP32 will send the I2S audio to the DAC of the codec. The DAC is connected to the HP outputs.

  Development platform used:
  SparkFun ESP32 IoT Redboard v10

  HARDWARE CONNECTIONS

  **********************
  ESP32 ------- CODEC
  **********************
  QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
  GND --------- GND         *optional, but not a bad idea
  5V ---------- VIN         *needed for source of codec's onboard AVDD (3.3V vreg)
  4 ----------- DDT         *aka DAC_DATA/I2S_SDO/"serial data out", this carries the I2S audio data from ESP32 to codec DAC
  16 ---------- BCK         *aka BCLK/I2S_SCK/"bit clock", this is the clock for I2S audio, can be conntrolled via controller or peripheral.
  25 ---------- DLRC        *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.

  **********************
  CODEC -------- AUDIO OUT
  **********************
  OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" connection for headphone output (aka "HP GND")
  HPL ---------- TRS OUTPUT TIP             *left HP output
  HPR ---------- TRS OUTPUT RING1           *right HP output

  Note, once connected and playing a sound file, your bluetooth source device (i.e. your phone) can control volume with its own volume control.

  You can also control the volume of the codecs built in headphone amp using this fuction:

  codec.setHeadphoneVolume(120); Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB

  Pete Lewis @ SparkFun Electronics
  October 14th, 2022
  https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library
  
  This code was created using some code by Mike Grusin at SparkFun Electronics
  Included with the LilyPad MP3 example code found here:
  Revision history: version 1.0 2012/07/24 MDG Initial release
  https://github.com/sparkfun/LilyPad_MP3_Player

  This code was created using some modified code from Phil Schatzmann's Arduino Library:
  https://github.com/pschatzmann/ESP32-A2DP
  The I2S configuration information about custom setups in the readme was super helpful to get I2S working.
  https://github.com/pschatzmann/ESP32-A2DP#readme
  This example is most similar to Phil Schatzmann's "bt_music_receiver_simple.ino" example.
  You can find that original code here:  
  https://github.com/pschatzmann/ESP32-A2DP/blob/main/examples/bt_music_receiver_simple/bt_music_receiver_simple.ino"
  Although, here we are setting up custom I2S pins and I2S modes to work for our needs.
  And we are setting up the matching I2S configuration on the WM8960.

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

#include "BluetoothA2DPSink.h" // Download library here: https://github.com/pschatzmann/ESP32-A2DP
BluetoothA2DPSink a2dp_sink;

// Connections to I2S bus (on the IoT Redboard)
#define I2S_WS 25
#define I2S_SD 17 // Note, this is not needed for this example, as it only sends data out to the codec's DAC (via I2S_SDO)
#define I2S_SDO 4
#define I2S_SCK 16

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 9 - Bluetooth Audio");

  Wire.begin();

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); // Freeze
  }
  Serial.println("Device is connected properly.");

  codec_setup();

  // Set up I2S
  i2s_install();
  i2s_setpin();

  a2dp_sink.start("myCodec"); // Note, you can give your device any name you'd like!
}

void loop()
{
 // Nothing do do here, but you can add more code if you like!
}

void codec_setup()
{
  // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  // Connect from DAC outputs to output mixer
  codec.enableLD2LO();
  codec.enableRD2RO();

  // Set gainstage between booster mixer and output mixer
  // For this loopback example, we are going to keep these as low as they go
  codec.setLB2LOVOL(0); // 0 = -21dB
  codec.setRB2ROVOL(0); // 0 = -21dB

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
  codec.setWL(WM8960_WL_16BIT);

  codec.enablePeripheralMode();
  //codec.enableMasterMode();
  //codec.setALRCGPIO(); // Note, should not be changed while ADC is enabled.

  // Enable DACs
  codec.enableDacLeft();
  codec.enableDacRight();

  //codec.enableLoopBack(); // Loopback sends ADC data directly into DAC
  codec.disableLoopBack();
  codec.disableDacMute(); // Default is "soft mute" on, so we must disable mute to make channels active

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolume(120);

  Serial.println("Codec Setup complete. Connect via Bluetooth, play music, and listen on Headphone outputs.");
}

void i2s_install() {
  // Set up I2S Processor configuration
  static i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100, // Updated automatically by A2DP
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = 0, // Default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true,
    .tx_desc_auto_clear = true // Avoiding noise in case of data unavailability
  };

  a2dp_sink.seti2s_config(i2s_config);
}

void i2s_setpin() {
  // Set I2S pin configuration
    i2s_pin_config_t my_pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SDO,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  a2dp_sink.setpin_config(my_pin_config);
}