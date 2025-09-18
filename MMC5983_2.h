#ifndef __MMC5983_H_
#define __MMC5983_H_
#include <stdint.h>
#include <stdbool.h>
#include "io.h"

//**************** SPI R/W bit******************//
#define SPI_R                   0x80
#define SPI_W                   0x00

//寄存器地址
#define XOUT0                   0x00
#define XOUT1                   0x01
#define YOUT0                   0x02
#define YOUT1                   0x03
#define ZOUT0                   0x04
#define ZOUT1                   0x05
#define XYZOUT2                 0x06
#define TOUT                    0x07
#define MMC_STATUS              0x08
#define INTERNAL_CTRL0          0x09
#define INTERNAL_CTRL1          0x0A
#define INTERNAL_CTRL2          0x0B
#define INTERNAL_CTRL3          0x0C
#define MMC5983_ADDR            0x2F

/*control 0*/
#define TM_M_START              1U
#define TM_T_START              (1U)<<1
#define TM_T_SET                (1U)<<3

/*control 1*/
//Register 1 bit 01 
#define RS_8MS_100Hz            0U
#define RS_4MS_200Hz            1U
#define RS_2MS_400Hz            2U
#define RS_0_5MS_800Hz          3U

#define SW_RST                  (1U<<7)


/*//control 2 */
//采样频率
#define CM_FREQ_OFF             0U
#define CM_FREQ_1Hz             1U
#define CM_FREQ_10Hz            2U
#define CM_FREQ_20Hz            3U
#define CM_FREQ_50Hz            4U
#define CM_FREQ_100Hz           5U
#define CM_FREQ_200Hz           6U
#define CM_FREQ_1000Hz          7U
//200 配合BW01 1000配合BW11

//测量模式 0单次；1连续
#define CMM_EN_SINGLE           (0U<<3)
#define CMM_EN_CONTINUS         (1U<<3)

//采样次数
#define PRD_SET_1TIMES          (0U<<4) 
#define PRD_SET_25TIMES         (1U<<4) 
#define PRD_SET_75TIMES         (2U<<4) 
#define PRD_SET_100TIMES        (3U<<4) 
#define PRD_SET_250TIMES        (4U<<4) 
#define PRD_SET_500TIMES        (5U<<4) 
#define PRD_SET_1000TIMES       (6U<<4) 
#define PRD_SET_2000TIMES       (7U<<4) 

#define EN_PRD_SET              (1U<<7)

#define MMC5983_ID              0x30

/*结构体存储区*/
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
}mmc_5983_data;

uint8_t read_mmc5983_id(void);

void mmc5983_init(void);

void mmc5983_data_read(mmc_5983_data* mag);

uint8_t mmc5983_status_read(void);

uint16_t mmc5983_read_temp(void);

#endif


