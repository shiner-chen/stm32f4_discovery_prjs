/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"


static uint16_t PDMBuffer[INTERNAL_BUFF_SIZE/2];
static uint16_t RecBuf[PCM_OUT_SIZE*2];/* PCM stereo samples are saved in RecBuf */
static uint8_t pHeaderBuff[44];

static FIL WaveFile;
/* Variable used by FatFs*/
static FIL FileRead;
//static DIR Directory;
static FILINFO FileInfo;

/* Variable used to indicate audio mode (play, record or stop). */
/* Defined in main.c */
extern __IO uint32_t CmdIndex;

/* Audio PDM remaining data length to be converted */
static __IO uint32_t AudioRemSize = 0;

/* Audio PDM data length to be played */
static uint32_t PdmDataLength = 0;

static WAVE_FormatTypeDef WaveFormat;

static Audio_BufferTypeDef  BufferCtl;

/* Private functions ---------------------------------------------------------*/
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t *pHeader);
static uint32_t WavProcess_HeaderInit(uint8_t *pHeader, WAVE_FormatTypeDef *pWaveFormatStruct);
static uint32_t WavProcess_HeaderUpdate(uint8_t *pHeader, WAVE_FormatTypeDef *pWaveFormatStruct);

/**
  * @brief  Encoder initialization.
  * @param  Freq: Sampling frequency.
  * @param  pHeader: Pointer to the WAV file header to be written.  
  * @retval 0 if success, !0 else.
  */
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t* pHeader)
{  
  /* Initialize the encoder structure */
  WaveFormat.SampleRate = Freq;        /* Audio sampling frequency */
  WaveFormat.NbrChannels = 2;          /* Number of channels: 1:Mono or 2:Stereo */
  WaveFormat.BitPerSample = 16;        /* Number of bits per sample (16, 24 or 32) */
  WaveFormat.FileSize = 0x001D4C00;    /* Total length of useful audio data (payload) */
  WaveFormat.SubChunk1Size = 44;       /* The file header chunk size */
  WaveFormat.ByteRate = (WaveFormat.SampleRate * \
                        (WaveFormat.BitPerSample/8) * \
                         WaveFormat.NbrChannels);     /* Number of bytes per second  (sample rate * block align)  */
  WaveFormat.BlockAlign = WaveFormat.NbrChannels * \
                         (WaveFormat.BitPerSample/8); /* channels * bits/sample / 8 */
  
  /* Parse the wav file header and extract required information */
  if(WavProcess_HeaderInit(pHeader, &WaveFormat))
  {
    return 1;
  }
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeader[0] = 'R';
  pHeader[1] = 'I';
  pHeader[2] = 'F';
  pHeader[3] = 'F';
  
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the 
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = 0x00;
  pHeader[5] = 0x4C;
  pHeader[6] = 0x1D;
  pHeader[7] = 0x00;
  
  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pHeader[8]  = 'W';
  pHeader[9]  = 'A';
  pHeader[10] = 'V';
  pHeader[11] = 'E';
  
  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pHeader[12]  = 'f';
  pHeader[13]  = 'm';
  pHeader[14]  = 't';
  pHeader[15]  = ' ';
  
  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pHeader[16]  = 0x10;
  pHeader[17]  = 0x00;
  pHeader[18]  = 0x00;
  pHeader[19]  = 0x00;
  
  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pHeader[20]  = 0x01;
  pHeader[21]  = 0x00;
  
  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pHeader[22]  = pWaveFormatStruct->NbrChannels;
  pHeader[23]  = 0x00;
  
  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pHeader[24]  = (uint8_t)((pWaveFormatStruct->SampleRate & 0xFF));
  pHeader[25]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 8) & 0xFF);
  pHeader[26]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 16) & 0xFF);
  pHeader[27]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 24) & 0xFF);
  
  /* Write the Byte Rate -----------------------------------------------------*/
  pHeader[28]  = (uint8_t)((pWaveFormatStruct->ByteRate & 0xFF));
  pHeader[29]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 8) & 0xFF);
  pHeader[30]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 16) & 0xFF);
  pHeader[31]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 24) & 0xFF);
  
  /* Write the block alignment -----------------------------------------------*/
  pHeader[32]  = pWaveFormatStruct->BlockAlign;
  pHeader[33]  = 0x00;
  
  /* Write the number of bits per sample -------------------------------------*/
  pHeader[34]  = pWaveFormatStruct->BitPerSample;
  pHeader[35]  = 0x00;
  
  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pHeader[36]  = 'd';
  pHeader[37]  = 'a';
  pHeader[38]  = 't';
  pHeader[39]  = 'a';
  
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pHeader[40]  = 0x00;
  pHeader[41]  = 0x4C;
  pHeader[42]  = 0x1D;
  pHeader[43]  = 0x00;
  
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the 
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = (uint8_t)(BufferCtl.fptr);
  pHeader[5] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[6] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[7] = (uint8_t)(BufferCtl.fptr >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  BufferCtl.fptr -=44;
  pHeader[40] = (uint8_t)(BufferCtl.fptr); 
  pHeader[41] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[42] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[43] = (uint8_t)(BufferCtl.fptr >> 24); 
  /* Return 0 if all operations are OK */
  return 0;
}

void Pdm2Wave(void)
{ 
	UINT bytesread = 0;
	/* Open the Wave file to be played */
	if(f_open(&FileRead, REC_PDM_NAME , FA_READ) != FR_OK)
	{
		BSP_LED_On(LED5);
		CmdIndex = CMD_RECORD;
	}
	else
	{    
		/* Set PdmDataLenght to the Speech PDM length */
		f_stat (REC_PDM_NAME, &FileInfo);
		PdmDataLength = FileInfo.fsize;
		//f_lseek(&FileRead, 0);
	}    

  /* Current size of the recorded buffer */
  uint32_t byteswritten = 0; 
  
  /* Remove Wave file if it exists on USB Flash Disk */
  f_unlink (REC_WAVE_NAME);

  
  /* Open the file to write on it */
  if ((f_open(&WaveFile, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK))
  {   
    while(1)
    {
      /* Toggle LED5 in infinite loop to signal that: USB Flash Disk is not connected/removed
         or an issue has occurred when creating/opening Wave file */
      BSP_LED_Toggle(LED5); 
    }
  }
#if 1 
  /* Initialize header file */
  WavProcess_EncInit(DEFAULT_AUDIO_IN_FREQ, pHeaderBuff);
  
  /* Write the header Wave */
  f_write(&WaveFile, pHeaderBuff, 44, (void *)&byteswritten);
	
	BufferCtl.fptr = byteswritten;
  BufferCtl.offset = BUFFER_OFFSET_NONE;
#endif
	AudioRemSize = PdmDataLength;
  
	while(AudioRemSize > 0)
	{
		f_read (&FileRead, &PDMBuffer[0], INTERNAL_BUFF_SIZE, &bytesread);

		/* PDM to PCM data convert */
		BSP_AUDIO_IN_PDMToPCM((uint16_t*)&PDMBuffer[0], (uint16_t*)&RecBuf[0]);
  
		if(f_write(&WaveFile, (uint8_t*)RecBuf, PCM_OUT_SIZE*4, (void*)&byteswritten) != FR_OK)
		{
			Error_Handler();
		}
#if 1
    BufferCtl.fptr += byteswritten;
#endif
    if(AudioRemSize > (INTERNAL_BUFF_SIZE))
    {
			AudioRemSize -= bytesread;
    }
    else
    {
      AudioRemSize = 0;
		}
		//BSP_LED_Toggle(LED4); 
	}
#if 1
  /* Update the data length in the header of the recorded Wave */    
  f_lseek(&WaveFile, 0);
  
  /* Parse the wav file header and extract required information */
  WavProcess_HeaderUpdate(pHeaderBuff, &WaveFormat);
  f_write(&WaveFile, pHeaderBuff, 44, (void*)&byteswritten);
#endif
  /* Close file and unmount MyFilesystem */
  f_close (&WaveFile);
  f_close (&FileRead);
}
