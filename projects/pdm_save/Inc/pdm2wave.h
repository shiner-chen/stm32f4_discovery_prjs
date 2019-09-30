/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PDM2WAVE_H
#define __PDM2WAVE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"


/* Exported types ------------------------------------------------------------*/
/* Exported Defines ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines for the Audio recording process */
#define WR_BUFFER_SIZE           4096  /*  More the size is higher, the recorded quality is better */ 
#define DEFAULT_TIME_REC         30000  /* Recording time in millisecond (Systick Time Base*TIME_REC= 1ms*30000)
                                        (default: 30s) */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void      Pdm2Wave(void);
#endif /* __PDM2WAVE_H */

