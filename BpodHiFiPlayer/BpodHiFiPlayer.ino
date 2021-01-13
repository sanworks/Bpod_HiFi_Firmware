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

#define MAX_WAVEFORMS 20
#define NBYTES_PER_SAMPLE 4 // 16 bit stereo
#define MAX_SECONDS_PER_WAVEFORM 10
#define MAX_SAMPLING_RATE 192000
#define MAX_ENVELOPE_SIZE 2000 // Maximum size of AM onset/offset envelope (in samples)
#define FILE_TRANSFER_BUFFER_SIZE 64000
#define FILE_TRANSFER_BUFFER_SIZE_SAMPLES FILE_TRANSFER_BUFFER_SIZE/4
#define USB_TRANSFER_BUFFER_SIZE 10000
#define MAX_MEMORY_BYTES FILE_TRANSFER_BUFFER_SIZE*MAX_WAVEFORMS

// --- TI PCM5122 DAC macros ---

#define PCM5122_ADDRESS 0x4D

// --- TI TPA6130A2 Audio Amp macros ---
#define TPA6130A2_ADDRESS 0x60
#define TPA6130A2_REG1 0x1
#define TPA6130A2_REG2 0x2

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
uint32_t playBufferPos = 0; // Position of current sample in the current data buffer
uint32_t ramBufferPlaybackPos = 0;
uint32_t playbackFilePos = 0; // Position of current sample in the data file being played from microSD -> DAC
byte currentPlayBuffer = 0; // Current buffer for each channel (a double buffering scheme allows one to be filled while the other is read)
byte PowerState = 0;
boolean schedulePlaybackStop = false;

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
  CodecDAC_begin();
  Wire.begin();
  setAmpPower(false); // Disable headphone amp
  setAmpGain(headphoneAmpGain); 
  setupDAC();
  
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
          USBCOM.writeUint32(samplingRate);
          USBCOM.writeUint32(MAX_WAVEFORMS);
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
        CodecDAC_config_i2s();
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
        playingFromRamBuffer = true;
        isActiveInterrupt = true;
        currentLoopMode = loopMode[waveIndex];
        if (useAMEnvelope) {
          inEnvelope = true;
          envelopeDir = 0;
          envelopePos = 0;
        }
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
  if (isPlaying && sdLoadFlag) {
    sdLoadFlag = false;
    Wave0.seek(playbackFilePos);
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

void CodecDAC_config_i2s(void)
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

void CodecDAC_begin(void)
{
  CodecDAC_dma.begin(true); // Allocate the DMA channel first
  CodecDAC_config_i2s();
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

  CodecDAC_dma.attachInterrupt(CodecDAC_isr);
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
  }
  arm_dcache_flush_delete(myi2s_tx_buffer.int32, sizeof(myi2s_tx_buffer.int32));
  CodecDAC_dma.clearInterrupt();
  isActiveInterrupt = 1 - isActiveInterrupt;
  if (schedulePlaybackStop) {
    isPlaying = false;
    schedulePlaybackStop = false;
  }
}

bool sdBusy() {
  return ready ? SDcard.card()->isBusy() : false;
}

void setAmpPower(boolean powerState) {
  if (powerState == 0) {
    TPA6130A2_write(TPA6130A2_REG1, 0); // 00000000 - both channels disabled
  } else {
    TPA6130A2_write(TPA6130A2_REG1, 192); // 11000000 - both channels enabled
  }
}

void setAmpGain(byte taperLevel) {
  if (taperLevel > 63) {
    taperLevel = 63;
  }
  TPA6130A2_write(TPA6130A2_REG2, taperLevel);
}

void setupDAC() {
  PCM5122_write(2, 0x10); // Power --> Standby
  PCM5122_write(1, 0x11); // Reset DAC
  PCM5122_write(2, 0x00); // Power --> On
  PCM5122_write(13, 0x10); // Set PLL reference clock = BCK
  PCM5122_write(14, 0x10); // Set DAC clock source = PLL
  PCM5122_write(37, 0x7D); // Ignore various errors
  PCM5122_write(61, 48); // Set left ch volume attenuation = 0dB
  PCM5122_write(62, 48); // Set right ch volume attenuation = 0dB
  PCM5122_write(65, 0x00); // Set mute off
  PCM5122_write(2, 0x00); // Power --> On
}

void setStartSamplePositions() {
  for (int i = 0; i < MAX_WAVEFORMS; i++) {
    waveformStartSamplePosSD[i] = i * samplingRate * MAX_SECONDS_PER_WAVEFORM;
    waveformStartSamplePosRAM[i] = i*FILE_TRANSFER_BUFFER_SIZE_SAMPLES;
  }
}

void TPA6130A2_write(byte address, byte val) {
  Wire.beginTransmission(TPA6130A2_ADDRESS);  // start transmission to device   
  Wire.write(address);                 // send register address
  Wire.write(val);                     // send value to write
  Wire.endTransmission();               // end transmission
}

void PCM5122_write(byte address, byte val) {
  Wire.beginTransmission(PCM5122_ADDRESS);  // start transmission to device   
  Wire.write(address);                 // send register address
  Wire.write(val);                     // send value to write
  Wire.endTransmission();               // end transmission
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
