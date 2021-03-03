/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_HiFi_Firmware repository
  Copyright (C) 2021 Sanworks LLC, Rochester, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// Note: Requires Arduino 1.8.13 or newer, and Teensyduino 1.5.4 (tested on 1.5.4 beta 5)

#include <Audio.h>
#include <utility/imxrt_hw.h>
#include <Wire.h>
#include <SPI.h>
#include "ArCOM.h"
#include "SdFat.h"

#define FirmwareVersion 1
#define TEENSY_I2S_MASTER 0 // Set to 1 for Teensy as I2S master. Set to 0 for HiFiBerry as I2S master using its precision clock source. Set to 0 for DAC2 Pro, DAC2 HD
#define IS_HD 1 // 0 for HiFiBerry DAC2 Pro, 1 for DAC2 HD

#define BIT_DEPTH 16 // Bits per sample
#define MAX_SAMPLING_RATE 192000 // Hz
#define MAX_WAVEFORMS 20 // Number of separate audio waveforms the device can store
#define MAX_SECONDS_PER_WAVEFORM 20 // Maximum number of seconds per waveform
#define MAX_ENVELOPE_SIZE 2000 // Maximum size of AM onset/offset envelope (in samples)
#define SYNC_PIN 30 // GPIO pin controlling SYNC BNC output
#define SYNC_PIN_DELAY_ONSET 22 // Number of DMA ISR calls before sync line goes high after sound onset
#define SYNC_PIN_DELAY_OFFSET 27 // Number of DMA ISR calls before sync line goes low after sound end
#define FILE_TRANSFER_BUFFER_SIZE 64000
#define USB_TRANSFER_BUFFER_SIZE 10000

#define FILE_TRANSFER_BUFFER_SIZE_SAMPLES FILE_TRANSFER_BUFFER_SIZE/4
#define MAX_MEMORY_BYTES FILE_TRANSFER_BUFFER_SIZE*MAX_WAVEFORMS
#define NBYTES_PER_SAMPLE BIT_DEPTH/4 // 4 = 16 bit stereo, 8 = 24 bit stereo (encoded with 32 bit ints)

// --- TI PCM5122 DAC macros ---

#define PCM5122_ADDRESS 0x4D

// --- TI PCM1796 DAC macros ---
#define PCM1796_ADDRESS 0x4C
#define SI5351_ADDRESS 0x62

// --- TI TPA6130A2 Audio Amp macros ---
#define TPA6130A2_ADDRESS 0x60
#define TPA6130A2_REG1 0x1
#define TPA6130A2_REG2 0x2

byte resetPin = 33;

// microSD setup
SdFs SDcard;
FsFile Wave0; // File on microSD card, to store waveform data
byte fileTransferBuffer[FILE_TRANSFER_BUFFER_SIZE] = {0};
bool ready = false;

// USB Transfer
byte usbTransferBuffer[USB_TRANSFER_BUFFER_SIZE] = {0};

// Headphone amp setup
boolean useHeadphoneAmp = false;
byte headphoneAmpGain = 52; // Range = 0-63. 52 = –0.3dB attenuation (closest position to 0). See table 2 on p.17 of TPA6130A2 datasheet for exact attenuation at each position

// Loop Mode
byte loopMode[MAX_WAVEFORMS] = {0}; // (for each sound) Loops waveform until loopDuration seconds
uint32_t loopDuration[MAX_WAVEFORMS] = {0}; // Duration of loop for loop mode (in samples). 0 = loop until canceled
boolean currentLoopMode = false; // Loop mode, for the currently triggered sound
uint32_t playbackTime = 0; // Time (in samples) since looping channel was triggered (used to compute looped playback end)

// Module setup
char moduleName[] = "HiFi"; // Name of module for manual override UI and state machine assemble


DMAMEM __attribute__((aligned(32))) static union {
  int16_t int16[2];
  int32_t int32[1];
} myi2s_tx_buffer;
static DMAChannel CodecDAC_dma;

// microSD Data Transfer Vars
unsigned long nFullReads = 0;
unsigned long partialReadSize = 0;

ArCOM USBCOM(Serial);
ArCOM StateMachineCOM(Serial1);

boolean LED_Enabled = true;

byte state = 0;
byte stateCount = 0;
byte opCode = 0;
byte opSource = 0;
byte waveIndex = 0;
uint32_t nSamples[MAX_WAVEFORMS] = {0};
uint32_t nWaveformBytes[MAX_WAVEFORMS] = {0}; //Number of bytes in each waveform
boolean newWaveLoaded[MAX_WAVEFORMS] = {false}; // True if a new sound was loaded; on next trial start signal, currentLoadBuffer is swapped.
uint32_t waveformStartSamplePosSD[MAX_WAVEFORMS] = {0};
uint32_t waveformEndSamplePosSD[MAX_WAVEFORMS] = {0};
uint32_t waveformStartSamplePosRAM[MAX_WAVEFORMS] = {0};

uint32_t currentPlaybackPos = 0;
uint32_t bufferPosL = 0;
uint32_t bufferPosR = 0;
uint32_t endPos = 0;
uint32_t samplingRate = MAX_SAMPLING_RATE;
boolean isPlaying = false;
boolean playingFromRamBuffer = false;
boolean isActiveInterrupt = false;
boolean sdLoadFlag = false;
boolean sdStartFlag = false;
uint32_t playBufferPos = 0; // Position of current sample in the current data buffer
uint32_t ramBufferPlaybackPos = 0;
uint32_t playbackFilePos = 0; // Position of current sample in the data file being played from microSD -> DAC
byte currentPlayBuffer = 0; // Current buffer for each channel (a double buffering scheme allows one to be filled while the other is read)
byte PowerState = 0;
boolean schedulePlaybackStop = false;
boolean syncPinStartFlag = false; // True if a trigger has been received to start audio playback
boolean syncPinEndFlag = false;
uint32_t syncPinStartTimer = 0;
uint32_t syncPinEndTimer = 0;

// AM onset/offset envelope
boolean useAMEnvelope = false; // True if using AM envelope on sound start/stop
boolean inEnvelope = false; // True while playing the envelope, false during remaining playback samples
boolean inFadeOut = false; // True if playback has been stopped, and envelope is playing in reverse (i.e. Fade-out)
uint16_t envelopeSize = MAX_ENVELOPE_SIZE;
uint16_t envelopePos = 0; // Current position in envelope
boolean envelopeDir = 0; // Current envelope playback direction 0 = forward (for sound onset), 1 = reverse (for sound offset)
union { // Floating point amplitude in range 1 (max intensity) -> 0 (silent). Data in forward direction.
    byte byteArray[MAX_ENVELOPE_SIZE*4];
    float floatArray[MAX_ENVELOPE_SIZE];
} AMenvelope; // Default hardware timer period during playback, in microseconds (100 = 10kHz). Set as a Union so it can be read as bytes.


EXTMEM union {
  byte byteArray[MAX_MEMORY_BYTES];
  int16_t int16[MAX_MEMORY_BYTES / 2];
  int32_t int32[MAX_MEMORY_BYTES / 4];
} AudioData;

union {
    byte byteArray[FILE_TRANSFER_BUFFER_SIZE];
    int16_t int16[FILE_TRANSFER_BUFFER_SIZE/2];
    int32_t int32[FILE_TRANSFER_BUFFER_SIZE/4];
} BufferA; // channel1BufferA and channel1BufferB form a double-buffer - one buffer fills while the other drives the DAC
union {
    byte byteArray[FILE_TRANSFER_BUFFER_SIZE];
    int16_t int16[FILE_TRANSFER_BUFFER_SIZE/2];
    int32_t int32[FILE_TRANSFER_BUFFER_SIZE/4];
} BufferB; 

uint32_t bufferPlaybackPos = 0;

void setup() {
  pinMode(SYNC_PIN, OUTPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  digitalWrite(resetPin, LOW);
  CodecDAC_begin();
  Wire.begin();
  setAmpPower(false); // Disable headphone amp
  setAmpGain(headphoneAmpGain);
  #if IS_HD
    setup_SI5351();
    setup_PCM1796();
  #else
    #if TEENSY_I2S_MASTER 
      setup_PCM5122_I2SSlave();
    #else
      setup_PCM5122_I2SMaster();
    #endif
  #endif
  
  Serial1.begin(1312500);
  SDcard.begin(SdioConfig(FIFO_SDIO));
  Wave0 = SDcard.open("Wave0.wfm", O_RDWR | O_CREAT);
  Wave0.seek(waveformStartSamplePosSD[waveIndex] * 4);
  while (sdBusy()) {}

  // Clear AudioData (initialized to random values)
  for (int i = 0; i < MAX_MEMORY_BYTES / 4; i++) {
    AudioData.int32[i] = 0;
  }
  setStartSamplePositions();
}

void loop() {
  opCode = 0;
  if (USBCOM.available() > 0) {
    opCode = USBCOM.readByte();
    opSource = 0;
  } else if (StateMachineCOM.available() > 0) {
    opCode = StateMachineCOM.readByte();
    opSource = 1;
  }
  if (opCode > 0) {
    switch (opCode) {
      case 255: // Return Bpod module info
        if (opSource == 1) { // Only returns this info if requested from state machine device
          returnModuleInfo();
        }
        break;
      case 243: // Return ack to PC
        if (opSource == 0) { // Only returns this info if requested from PC
          USBCOM.writeByte(244);
        }
      break;
      case 'I': // Return info to PC
        if (opSource == 0) {
          USBCOM.writeByte(TEENSY_I2S_MASTER);
          USBCOM.writeByte(BIT_DEPTH);
          USBCOM.writeByte(MAX_WAVEFORMS);
          USBCOM.writeUint32(samplingRate);
          USBCOM.writeUint32(MAX_SECONDS_PER_WAVEFORM);
          USBCOM.writeUint32(MAX_ENVELOPE_SIZE);
        }
      break;
      case 'E': // Use/disuse AM envelope
        useAMEnvelope = USBCOM.readByte();
        USBCOM.writeByte(1); // Acknowledge
        envelopePos = 0;
        envelopeDir = 0;
      break;
      case 'M': // Load AM envelope
          envelopeSize = USBCOM.readUint16();
          if (envelopeSize <= MAX_ENVELOPE_SIZE) {
            USBCOM.readByteArray(AMenvelope.byteArray, envelopeSize*4);
            USBCOM.writeByte(1); // Acknowledge
          } else {
            USBCOM.writeByte(0); // Acknowledge
          }
      break;
      case 'O': // Set loop mode (for each channel)
        if (opSource == 0){
          USBCOM.readByteArray(loopMode, MAX_WAVEFORMS);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;
      case '-': // Set loop duration (for each channel)
        if (opSource == 0){
          USBCOM.readUint32Array(loopDuration,MAX_WAVEFORMS);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;
      case 'H': // Enable headphone amp
        useHeadphoneAmp = USBCOM.readByte();
        setAmpPower(useHeadphoneAmp);
        USBCOM.writeByte(1); // Acknowledge
      break;
      case 'G': // Set headphone amp gain
        headphoneAmpGain = USBCOM.readByte();
        setAmpGain(headphoneAmpGain);
        USBCOM.writeByte(1); // Acknowledge
      break;
      case 'S': // Set sampling rate
        samplingRate = USBCOM.readUint32();
        #if TEENSY_I2S_MASTER 
          CodecDAC_config_i2s_master();
        #else
          #if IS_HD
            set_PCM1796_SF();
          #else
            setup_PCM5122_I2SMaster();
          #endif
        #endif
        setStartSamplePositions();
        USBCOM.writeByte(1); // Acknowledge
      break;
      case 'L':
        if (opSource == 0) {
          waveIndex = USBCOM.readByte();
          if (waveIndex < MAX_WAVEFORMS) { // Sanity check
            nSamples[waveIndex] = USBCOM.readUint32();
            nWaveformBytes[waveIndex] = nSamples[waveIndex] * 4;
            Wave0.seek(waveformStartSamplePosSD[waveIndex] * 4);
            while (sdBusy()) {}
            nFullReads = (unsigned long)(floor((double)nWaveformBytes[waveIndex] / (double)FILE_TRANSFER_BUFFER_SIZE));
            for (int i = 0; i < nFullReads; i++) {
              while (USBCOM.available() == 0) {}
              if (i == 0) {
                USBCOM.readByteArray(AudioData.byteArray + (waveformStartSamplePosRAM[waveIndex]*4), FILE_TRANSFER_BUFFER_SIZE);
              } else {
                USBCOM.readByteArray((char*)fileTransferBuffer, FILE_TRANSFER_BUFFER_SIZE);
                Wave0.write(fileTransferBuffer, FILE_TRANSFER_BUFFER_SIZE);
                while (sdBusy()) {}
              }
            }
            partialReadSize = (nWaveformBytes[waveIndex]) - (nFullReads * FILE_TRANSFER_BUFFER_SIZE);
            if (partialReadSize > 0) {
              if (nFullReads == 0) {
                USBCOM.readByteArray(AudioData.byteArray + (waveformStartSamplePosRAM[waveIndex]*4), partialReadSize);
              } else {
                USBCOM.readByteArray((char*)fileTransferBuffer, partialReadSize);
                Wave0.write(fileTransferBuffer, partialReadSize);
                while (sdBusy()) {}
              }
            }
            USBCOM.writeByte(1); Serial.send_now();
            newWaveLoaded[waveIndex] = true;
          }
          waveformEndSamplePosSD[waveIndex] = (waveformStartSamplePosSD[waveIndex] + nSamples[waveIndex]);
        }
        break;
      case 'P':
        switch (opSource) {
          case 0:
            waveIndex = USBCOM.readByte();
            break;
          case 1:
            waveIndex = StateMachineCOM.readByte();
            break;
        }
        if (opSource == 1) {
          StateMachineCOM.writeByte(1);
        }
        currentPlaybackPos = waveformStartSamplePosSD[waveIndex];
        ramBufferPlaybackPos = waveformStartSamplePosRAM[waveIndex];
        currentPlayBuffer = 1;
        bufferPlaybackPos = 0;
        playbackTime = 0;
        playbackFilePos = currentPlaybackPos * 4; // Sound start position on microSD card in bytes
        sdLoadFlag = true;
        sdStartFlag = true;
        playingFromRamBuffer = true;
        isActiveInterrupt = true;
        currentLoopMode = loopMode[waveIndex];
        if (useAMEnvelope) {
          inEnvelope = true;
          envelopeDir = 0;
          envelopePos = 0;
        }
        syncPinStartFlag = true;
        syncPinStartTimer = 0;
        isPlaying = true;
      break;
      case 'X':
        if (useAMEnvelope) {
          inEnvelope = true;
          envelopeDir = 1;
          envelopePos = 0;
        } else {
          isPlaying = false;
        }
      break;
      case 'Y': // Create and/or Clear data file on microSD card, with enough space to store all waveforms (to be optimized for speed)
        if (opSource == 0) {
          for (int i = 0; i < FILE_TRANSFER_BUFFER_SIZE; i++) {
            fileTransferBuffer[i] = 65;
          }
          Wave0.close();
          SDcard.remove("Wave0.wfm");
          Wave0 = SDcard.open("Wave0.wfm", O_RDWR | O_CREAT);
          Wave0.seek(0); // Set write position to first byte
          for (uint32_t i = 0; i < (MAX_WAVEFORMS * NBYTES_PER_SAMPLE * MAX_SAMPLING_RATE * MAX_SECONDS_PER_WAVEFORM * 2) / FILE_TRANSFER_BUFFER_SIZE; i++) {
            Wave0.write(fileTransferBuffer, FILE_TRANSFER_BUFFER_SIZE); // Write fileTransferBufferSize zeros
            ready = true;
            while (sdBusy()) {
              // Do something here.
            }
          }
          delayMicroseconds(100000);
          Wave0.close();
          Wave0 = SDcard.open("Wave0.wfm", O_RDWR | O_CREAT);
          USBCOM.writeByte(1); // Acknowledge
        }
        break;
    }
  }
  // MicroSD transfer
  if (sdStartFlag) {
    Wave0.seek(playbackFilePos);
    sdStartFlag = false;
  }
  if (isPlaying && sdLoadFlag) {
    sdLoadFlag = false;
    if (currentPlayBuffer == 1) {
      Wave0.read(BufferA.byteArray, FILE_TRANSFER_BUFFER_SIZE);
    } else {
      Wave0.read(BufferB.byteArray, FILE_TRANSFER_BUFFER_SIZE);
    }
    playbackFilePos += FILE_TRANSFER_BUFFER_SIZE;
  }  
}


FLASHMEM static void configure_audioClock(int nfact, int32_t nmult, uint32_t ndiv, bool force) // sets PLL4
{
  if (!force && (CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_ENABLE)) return;

  CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE
                         | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) // 2: 1/4; 1: 1/2; 0: 1/1
                         | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

  CCM_ANALOG_PLL_AUDIO_NUM   = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
  CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;

  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) {}; //Wait for pll-lock

  const int div_post_pll = 1; // other values: 2,4
  CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
  if (div_post_pll > 1) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
  if (div_post_pll > 3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;

  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;//Disable Bypass
}

void CodecDAC_config_i2s_master(void)
{
  CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON); //enables SAI1 clock in CCM_CCGR5 register

  //PLL:
  int fs = samplingRate;
  // PLL between 27*24 = 648MHz and 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);

  double C = ((double)fs * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  configure_audioClock(c0, c1, c2, false);

  // clear SAI1_CLK register locations
  CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
               | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4; first part clears all bits except SAI1_CLK_SEL; second part choosing PLL4 currently
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
               | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) // &0x07
               | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1); // &0x3f

  // Select MCLK
  IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 // master clock is an output and something else?
                     & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
                    | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));

  CORE_PIN23_CONFIG = 3;  //1:MCLK
  CORE_PIN21_CONFIG = 3;  //1:RX_BCLK
  CORE_PIN20_CONFIG = 3;  //1:RX_SYNC

  int rsync = 0;
  int tsync = 1;

  I2S1_TMR = 0; // no masking
  //I2S1_TCSR = (1<<25); //Reset
  I2S1_TCR1 = I2S_TCR1_RFW(1);
  I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP // sync=0; tx is async;
              | (I2S_TCR2_BCD | I2S_TCR2_DIV((1)) | I2S_TCR2_MSEL(1));
  I2S1_TCR3 = I2S_TCR3_TCE;
  I2S1_TCR4 = I2S_TCR4_FRSZ((2 - 1)) | I2S_TCR4_SYWD((32 - 1)) | I2S_TCR4_MF
              | I2S_TCR4_FSD | I2S_TCR4_FSE | I2S_TCR4_FSP;
  I2S1_TCR5 = I2S_TCR5_WNW((32 - 1)) | I2S_TCR5_W0W((32 - 1)) | I2S_TCR5_FBT((32 - 1));

  I2S1_RMR = 0;
  //I2S1_RCSR = (1<<25); //Reset
  I2S1_RCR1 = I2S_RCR1_RFW(1);
  I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_RCR2_BCP  // sync=0; rx is async;
              | (I2S_RCR2_BCD | I2S_RCR2_DIV((1)) | I2S_RCR2_MSEL(1));
  I2S1_RCR3 = I2S_RCR3_RCE;
  I2S1_RCR4 = I2S_RCR4_FRSZ((2 - 1)) | I2S_RCR4_SYWD((32 - 1)) | I2S_RCR4_MF
              | I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
  I2S1_RCR5 = I2S_RCR5_WNW((32 - 1)) | I2S_RCR5_W0W((32 - 1)) | I2S_RCR5_FBT((32 - 1));
}

void CodecDAC_config_i2s_slave(void)
{
  CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

  // if either transmitter or receiver is enabled, do nothing
  if (I2S1_TCSR & I2S_TCSR_TE) return;
  if (I2S1_RCSR & I2S_RCSR_RE) return;

  // not using MCLK in slave mode - hope that's ok?
  //CORE_PIN23_CONFIG = 3;  // AD_B1_09  ALT3=SAI1_MCLK
  CORE_PIN21_CONFIG = 3;  // AD_B1_11  ALT3=SAI1_RX_BCLK
  CORE_PIN20_CONFIG = 3;  // AD_B1_10  ALT3=SAI1_RX_SYNC
  IOMUXC_SAI1_RX_BCLK_SELECT_INPUT = 1; // 1=GPIO_AD_B1_11_ALT3, page 868
  IOMUXC_SAI1_RX_SYNC_SELECT_INPUT = 1; // 1=GPIO_AD_B1_10_ALT3, page 872

  // configure transmitter
  I2S1_TMR = 0;
  I2S1_TCR1 = I2S_TCR1_RFW(1);  // watermark at half fifo size
  I2S1_TCR2 = I2S_TCR2_SYNC(1) | I2S_TCR2_BCP;
  I2S1_TCR3 = I2S_TCR3_TCE;
  I2S1_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(31) | I2S_TCR4_MF
    | I2S_TCR4_FSE | I2S_TCR4_FSP | I2S_RCR4_FSD;
  I2S1_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

  // configure receiver
  I2S1_RMR = 0;
  I2S1_RCR1 = I2S_RCR1_RFW(1);
  I2S1_RCR2 = I2S_RCR2_SYNC(0) | I2S_TCR2_BCP;
  I2S1_RCR3 = I2S_RCR3_RCE;
  I2S1_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(31) | I2S_RCR4_MF
    | I2S_RCR4_FSE | I2S_RCR4_FSP;
  I2S1_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);
}

void CodecDAC_begin(void)
{
  CodecDAC_dma.begin(true); // Allocate the DMA channel first
  #if TEENSY_I2S_MASTER 
    CodecDAC_config_i2s_master();
  #else
    CodecDAC_config_i2s_slave();
  #endif
  CORE_PIN7_CONFIG  = 3;  //1:TX_DATA0 pin 7 on uP
  CodecDAC_dma.TCD->SADDR = myi2s_tx_buffer.int16; //source address
  CodecDAC_dma.TCD->SOFF = 2; // source buffer address increment per transfer in bytes
  CodecDAC_dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1); // specifies 16 bit source and destination
  CodecDAC_dma.TCD->NBYTES_MLNO = 2; // bytes to transfer for each service request///////////////////////////////////////////////////////////////////
  CodecDAC_dma.TCD->SLAST = -sizeof(myi2s_tx_buffer.int16); // last source address adjustment
  CodecDAC_dma.TCD->DOFF = 0; // increments at destination
  CodecDAC_dma.TCD->CITER_ELINKNO = sizeof(myi2s_tx_buffer.int16) / 2;
  CodecDAC_dma.TCD->DLASTSGA = 0; // destination address offset
  CodecDAC_dma.TCD->BITER_ELINKNO = sizeof(myi2s_tx_buffer.int16) / 2;
  CodecDAC_dma.TCD->CSR = DMA_TCD_CSR_INTMAJOR; //| DMA_TCD_CSR_INTHALF; // enables interrupt when transfers half and full complete SET TO 0 to disable DMA interrupts
  CodecDAC_dma.TCD->DADDR = (void *)((uint32_t)&I2S1_TDR0 + 2); // I2S1 register DMA writes to
  CodecDAC_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX); // i2s channel that will trigger the DMA transfer when ready for data
  CodecDAC_dma.enable();

  I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
  I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;

  CodecDAC_dma.attachInterrupt(CodecDAC_isr, 1); // Optional second argument = priority, default = 128
}

void CodecDAC_isr(void)
{
  if (isActiveInterrupt) {
    if (isPlaying) {
      if (inEnvelope) {
          if (playingFromRamBuffer) {
            myi2s_tx_buffer.int16[0] = AudioData.int16[ramBufferPlaybackPos*2];
            myi2s_tx_buffer.int16[1] = AudioData.int16[(ramBufferPlaybackPos*2)+1];
            ramBufferPlaybackPos++;
          } else {
            if (currentPlayBuffer == 0) {
              myi2s_tx_buffer.int16[0] = BufferA.int16[bufferPlaybackPos*2];
              myi2s_tx_buffer.int16[1] = BufferA.int16[(bufferPlaybackPos*2)+1];
            } else {
              myi2s_tx_buffer.int16[0] = BufferB.int16[bufferPlaybackPos*2];
              myi2s_tx_buffer.int16[1] = BufferB.int16[(bufferPlaybackPos*2)+1];
            }
            bufferPlaybackPos++;
          }
          if (envelopeDir == 0) {
            myi2s_tx_buffer.int16[0] = myi2s_tx_buffer.int16[0] * AMenvelope.floatArray[envelopePos];
            myi2s_tx_buffer.int16[1] = myi2s_tx_buffer.int16[1] * AMenvelope.floatArray[envelopePos];
          } else {
            myi2s_tx_buffer.int16[0] = myi2s_tx_buffer.int16[0] * AMenvelope.floatArray[envelopeSize-envelopePos-1];
            myi2s_tx_buffer.int16[1] = myi2s_tx_buffer.int16[1] * AMenvelope.floatArray[envelopeSize-envelopePos-1];
          }
          envelopePos++;
          if (envelopePos == envelopeSize) {
            inEnvelope = false;
            inFadeOut = false;
            if (envelopeDir == 1) {
              schedulePlaybackStop = true;
            }
          }
      } else {
        if (playingFromRamBuffer) {
          myi2s_tx_buffer.int32[0] = AudioData.int32[ramBufferPlaybackPos];
          ramBufferPlaybackPos++;
        } else {
          if (currentPlayBuffer == 0) {
            myi2s_tx_buffer.int32[0] = BufferA.int32[bufferPlaybackPos];
          } else {
            myi2s_tx_buffer.int32[0] = BufferB.int32[bufferPlaybackPos];
          }
          bufferPlaybackPos++;
        }
      }

      currentPlaybackPos++;
      if (currentPlaybackPos == (waveformStartSamplePosSD[waveIndex] + FILE_TRANSFER_BUFFER_SIZE_SAMPLES)) {
        playingFromRamBuffer = false;
        currentPlayBuffer = 1-currentPlayBuffer;
        sdLoadFlag = true;
      }
      if (bufferPlaybackPos == FILE_TRANSFER_BUFFER_SIZE_SAMPLES) {
        currentPlayBuffer = 1-currentPlayBuffer;
        bufferPlaybackPos = 0;
        sdLoadFlag = true;
      }
      playbackTime++; // Sample count for looping waves, insensitive to wave restart
      if (currentLoopMode) {
        if (!(loopDuration[waveIndex]==0)) {
          if (useAMEnvelope) {
            if (playbackTime == loopDuration[waveIndex] - envelopeSize - 1) {
              inEnvelope = true;
              envelopeDir = 1;
              envelopePos = 0;
            }
          } else {
            if (playbackTime > loopDuration[waveIndex]) {
              schedulePlaybackStop = true;
            }
          }
        }
        if (currentPlaybackPos == waveformEndSamplePosSD[waveIndex]) {
           currentPlaybackPos = waveformStartSamplePosSD[waveIndex];
           ramBufferPlaybackPos = waveformStartSamplePosRAM[waveIndex];
           currentPlayBuffer = 1;
           bufferPlaybackPos = 0;
           playbackFilePos = currentPlaybackPos * 4; // Sound start position on microSD card in bytes
           sdStartFlag = true;
           sdLoadFlag = true;
           playingFromRamBuffer = true;
           isActiveInterrupt = true;
        }
      } else {
        if (useAMEnvelope) {
          if (currentPlaybackPos == waveformEndSamplePosSD[waveIndex] - envelopeSize - 1) {
            inEnvelope = true;
            envelopeDir = 1;
            envelopePos = 0;
          }
        }
        if (currentPlaybackPos == waveformEndSamplePosSD[waveIndex]) {
          schedulePlaybackStop = true;
        }
      }
    } else {
      myi2s_tx_buffer.int32[0] = 0;
    }
  } else {
    if (syncPinStartFlag) {
      if (syncPinStartTimer > SYNC_PIN_DELAY_ONSET) {
        digitalWriteFast(SYNC_PIN, HIGH);
        syncPinStartFlag = false;
      } else {
        syncPinStartTimer++;
      }
    }
    if (syncPinEndFlag) {
      if (syncPinEndTimer > SYNC_PIN_DELAY_OFFSET) {
        digitalWriteFast(SYNC_PIN, LOW);
        syncPinEndFlag = false;
      } else {
        syncPinEndTimer++;
      }
    }
  }
  arm_dcache_flush_delete(myi2s_tx_buffer.int32, sizeof(myi2s_tx_buffer.int32));
  CodecDAC_dma.clearInterrupt();
  isActiveInterrupt = 1 - isActiveInterrupt;
  if (schedulePlaybackStop) {
    isPlaying = false;
    schedulePlaybackStop = false;
    syncPinEndFlag = true;
    syncPinEndTimer = 0;
  }
}

bool sdBusy() {
  return ready ? SDcard.card()->isBusy() : false;
}

void setAmpPower(boolean powerState) {
  if (powerState == 0) {
    i2c_write(TPA6130A2_ADDRESS, TPA6130A2_REG1, 0); // 00000000 - both channels disabled
  } else {
    i2c_write(TPA6130A2_ADDRESS, TPA6130A2_REG1, 192); // 11000000 - both channels enabled
  }
}

void setAmpGain(byte taperLevel) {
  if (taperLevel > 63) {
    taperLevel = 63;
  }
  i2c_write(TPA6130A2_ADDRESS, TPA6130A2_REG2, taperLevel);
}

void setup_PCM5122_I2SSlave() {
  i2c_write(PCM5122_ADDRESS, 2,  B00010000); // Power --> Standby
  i2c_write(PCM5122_ADDRESS, 1,  B00010001); // Reset DAC
  i2c_write(PCM5122_ADDRESS, 2,  B00000000); // Power --> On
  if (LED_Enabled) {
    i2c_write(PCM5122_ADDRESS, 8, 0x8); // Register 8 = GPIO enable = 8 = enable ch4 (LED Line)
    i2c_write(PCM5122_ADDRESS, 83, 0x02); // Reg 82 = GPIO ch4 config = 02 = output controlled by reg 86
    i2c_write(PCM5122_ADDRESS, 86, B00001000); // Reg 86 = GPIO write = 8 (line 4 high --> LED on)
  }
  i2c_write(PCM5122_ADDRESS, 13, B00010000); // Set PLL reference clock = BCK
  i2c_write(PCM5122_ADDRESS, 14, B00010000); // Set DAC clock source = PLL
  i2c_write(PCM5122_ADDRESS, 37, B01111101); // Ignore various errors
  i2c_write(PCM5122_ADDRESS, 61, B00111010); // Set left ch volume attenuation, -5dB
  i2c_write(PCM5122_ADDRESS, 62, B00111010); // Set right ch volume attenuation, -5dB
  i2c_write(PCM5122_ADDRESS, 65, B00000000); // Set mute off
  i2c_write(PCM5122_ADDRESS, 2,  B00000000); // Power --> On
}

void setup_PCM5122_I2SMaster() {
    // Reset and powerup
  i2c_write(PCM5122_ADDRESS, 2,  B00010000); // Power --> Standby
  i2c_write(PCM5122_ADDRESS, 1,  B00010001); // Reset DAC
  i2c_write(PCM5122_ADDRESS, 2,  B00000000); // Power --> On
  i2c_write(PCM5122_ADDRESS, 2,  B00010000); // Power --> Standby

  // DAC GPIO setup
  i2c_write(PCM5122_ADDRESS, 8,  B00101100); // Register 8 = GPIO enable = 24 = enable ch3 + 6
  i2c_write(PCM5122_ADDRESS, 82, B00000010); // Reg 82 = GPIO ch3 config = 02 = output controlled by reg 86
  i2c_write(PCM5122_ADDRESS, 83, B00000010); // Reg 82 = GPIO ch4 config = 02 = output controlled by reg 86
  i2c_write(PCM5122_ADDRESS, 85, B00000010); // Reg 85 = GPIO ch6 config = 02 = output controlled by reg 86
  i2c_write(PCM5122_ADDRESS, 86, B00000000); // Reg 86 = GPIO write = 00 (all lines low)

  // Master mode setup
  i2c_write(PCM5122_ADDRESS, 9,  B00010001); //BCK, LRCLK configuration = 10001 = set BCK and LRCK to outputs (master mode)
  i2c_write(PCM5122_ADDRESS, 12, B00000011); //Master mode BCK, LRCLK reset = 11 = BCK and LRCK clock dividers enabled
  i2c_write(PCM5122_ADDRESS, 40, B00000000); //I2S data format, word length = 0 = 16 bit I2S //**DOES NOT MATTER**
  i2c_write(PCM5122_ADDRESS, 37, B01111101); // Ignore various errors
  i2c_write(PCM5122_ADDRESS, 4,  B00000000); // Disable PLL = 0 = off
  i2c_write(PCM5122_ADDRESS, 14, B00110000); //DAC clock source selection = 011 = SCK clock
  
  switch (samplingRate) {
    case 44100:
        i2c_write(PCM5122_ADDRESS, 32, B00000111); //Master mode BCK divider
        i2c_write(PCM5122_ADDRESS, 33, B00111111); //Master mode LRCK divider
        if (LED_Enabled) {
          i2c_write(PCM5122_ADDRESS, 86, B00101000); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        } else {
          i2c_write(PCM5122_ADDRESS, 86, B00100000); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        }
    break;
    case 48000:
        i2c_write(PCM5122_ADDRESS, 32, B00000111); //Master mode BCK divider
        i2c_write(PCM5122_ADDRESS, 33, B00111111); //Master mode LRCK divider
        if (LED_Enabled) {
          i2c_write(PCM5122_ADDRESS, 86, B00001100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        } else {
          i2c_write(PCM5122_ADDRESS, 86, B00000100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        }
    break;
    case 96000:
        i2c_write(PCM5122_ADDRESS, 32, B00000011); //Master mode BCK divider
        i2c_write(PCM5122_ADDRESS, 33, B00111111); //Master mode LRCK divider
        if (LED_Enabled) {
          i2c_write(PCM5122_ADDRESS, 86, B00001100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        } else {
          i2c_write(PCM5122_ADDRESS, 86, B00000100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        }
    break;
    case 192000:
        i2c_write(PCM5122_ADDRESS, 32, B00000001); //Master mode BCK divider
        i2c_write(PCM5122_ADDRESS, 33, B00111111); //Master mode LRCK divider
        if (LED_Enabled) {
          i2c_write(PCM5122_ADDRESS, 86, B00001100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        } else {
          i2c_write(PCM5122_ADDRESS, 86, B00000100); // Reg 86 = GPIO write = 36 (line 3 high) // Enable precision clock
        }
    break;
  }
  
  i2c_write(PCM5122_ADDRESS, 19, B00000001); // Clock sync request
  i2c_write(PCM5122_ADDRESS, 19, B00000000); // Clock sync start

  // Finish startup
  i2c_write(PCM5122_ADDRESS, 61, B00110001); // Set left ch volume attenuation  = -0.5dB
  i2c_write(PCM5122_ADDRESS, 62, B00110001); // Set right ch volume attenuation = -0.5dB
  i2c_write(PCM5122_ADDRESS, 65, B00000000); // Set mute off
  i2c_write(PCM5122_ADDRESS, 2,  B00000000); // Power --> On
}

void setup_SI5351() {
  byte REGs[] = {0x02,0x03,0x07,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x1C,0x1D,0x1F,0x2A,0x2C,0x2F,0x30,0x31,0x32,0x34,0x37,0x38,0x39,0x3A,0x3B,0x3E,0x3F,0x40,0x41,0x5A,0x5B,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0xA2,0xA3,0xA4,0xB7,0xB1,0x1A,0x1B,0x1E,0x20,0x21,0x2B,0x2D,0x2E,0x33,0x35,0x36,0x3C,0x3D,0xB1};
  byte Vals[] = {0x53,0x00,0x20,0x00,0x0D,0x1D,0x0D,0x8C,0x8C,0x8C,0x8C,0x8C,0x2A,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x92,0xAC,0x0C,0x35,0xF0,0x09,0x50,0x02,0x10,0x40,0x01,0x22,0x80,0x22,0x46,0xAC};

  for (int i = 0; i < sizeof(REGs); i++) {
    i2c_write(SI5351_ADDRESS, REGs[i], Vals[i]);
  }
}

void setup_PCM1796() {
  digitalWrite(resetPin, HIGH);
  delay(1);
  digitalWrite(resetPin, LOW);
  delay(1);
  digitalWrite(resetPin, HIGH);
  delay(10);
  i2c_write(PCM1796_ADDRESS, 16, B11111110); // -0.5 DB
  i2c_write(PCM1796_ADDRESS, 17, B11111110); // -0.5 DB
  i2c_write(PCM1796_ADDRESS, 18, B11000000); 
  i2c_write(PCM1796_ADDRESS, 19, B00000000); 
}

void set_PCM1796_SF() {
  byte REGs[] = {0x1A,0x1B,0x1E,0x20,0x21,0x2B,0x2D,0x2E,0x33,0x35,0x36,0x3C,0x3D,0xB1};
  byte Vals_441[] = {0x3D,0x09,0xF3,0x13,0x75,0x04,0x11,0xE0,0x01,0x9D,0x00,0x42,0x7A,0xAC};
  byte Vals_48[] = {0x0C,0x35,0xF0,0x09,0x50,0x02,0x10,0x40,0x01,0x90,0x00,0x42,0x46,0xAC};
  byte Vals_96[] = {0x0C,0x35,0xF0,0x09,0x50,0x02,0x10,0x40,0x01,0x47,0x00,0x32,0x46,0xAC};
  byte Vals_192[] = {0x0C,0x35,0xF0,0x09,0x50,0x02,0x10,0x40,0x01,0x22,0x80,0x22,0x46,0xAC};
  for (int i = 0; i < sizeof(REGs); i++) {
    switch (samplingRate) {
      case 44100:
        i2c_write(SI5351_ADDRESS, REGs[i], Vals_441[i]);
      break;
      case 48000:
        i2c_write(SI5351_ADDRESS, REGs[i], Vals_48[i]);
      break;
      case 96000:
        i2c_write(SI5351_ADDRESS, REGs[i], Vals_96[i]);
      break;
      case 192000:
        i2c_write(SI5351_ADDRESS, REGs[i], Vals_192[i]);
      break;
    }
  }
}

void setStartSamplePositions() {
  for (int i = 0; i < MAX_WAVEFORMS; i++) {
    waveformStartSamplePosSD[i] = i * samplingRate * MAX_SECONDS_PER_WAVEFORM;
    waveformStartSamplePosRAM[i] = i*FILE_TRANSFER_BUFFER_SIZE_SAMPLES;
  }
}

void i2c_write(byte i2cAddress, byte address, byte val) {
  Wire.beginTransmission(i2cAddress);  
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void returnModuleInfo() {
  StateMachineCOM.writeByte(65); // Acknowledge
  StateMachineCOM.writeUint32(FirmwareVersion); // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName) - 1); // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName) - 1); // Module name
  StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
  StateMachineCOM.writeByte('#'); // Op code for: Number of behavior events this module can generate
  StateMachineCOM.writeByte(MAX_WAVEFORMS);
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
}
