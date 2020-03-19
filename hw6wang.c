#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "LSM9DS1.h"

#define PWM_RANGE 100
#define APB_CLOCK 250000000

#define DEN_PIN   18
#define CS_M_PIN  19
#define CS_AG_PIN 20

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))


struct pause_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool            done;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  volatile struct spi_register *spi;
  int                             pin;
  struct pause_flag *             pause;
  struct done_flag *              done;
};

struct key_thread_parameter
{
  struct done_flag *  done;
  struct pause_flag * pause1;
  struct pause_flag * pause2;
  struct pause_flag * pause3;
  struct pause_flag * pause4;
  struct pause_flag * pause5;
  struct pause_flag * pause6;
};

int  Tstep = 50;  /* PWM time resolution, number used for usleep(Tstep) */
int  Tlevel = 5;  /* repetition count of each light level, eg. repeat 12% light level for 5 times. */
int inc = 0;
int dec = 0;
int forw = 0; //forward = 1
int back = 0; //backward  = 1
int sb = 0; // shortbreak =1
int left = 0;
int right = 0;
int init = 1;
int quit = 0;
int mode = 1;
int needleft = 0;
int needright = 0;
int newtrip = 0;

//hw6 parameters
float acc[1200][3] ={};
float gy[1200][3] ={};
float mag[1200][3] ={};
int acci = 0;
int gyi = 0;
int magi = 0;
int readtime =0;
//range parameters
float maxnum[9] = {};
float minnum[9] = {}; 

float rangenum[9][9] = {};
float coor[1200][3] = {};
int maparr[20][20] = {};


void transact_SPI(                                /* send/receive SPI data */
    uint8_t const *                 write_data,   /* the data to send to a SPI device */
    uint8_t *                       read_data,    /* the data read from the SPI device */
    size_t                          data_length,  /* the length of data to send/receive */
    int                             CS_pin,       /* the pin to toggle when communicating */
    volatile struct gpio_register * gpio,         /* the GPIO address */
    volatile struct spi_register *  spi )         /* the SPI address */
{
  size_t  write_count;  /* used to index through the bytes to be sent/received */
  size_t  read_count;   /* used to index through the bytes to be sent/received */

  /* clear out anything in the RX FIFO */
  while (spi->CS.field.RXD != 0)
  {
    (void)spi->FIFO;
  }

  /* enable the chip select on the device */
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );

  /* see section 10.6.1 of the BCM2835 datasheet
   * Note that the loop below is a busy-wait loop and may burn a lot of clock cycles, depending on the amount of data to be transferred
   */
  spi->CS.field.TA = 1;
  write_count = 0;
  read_count  = 0;
  do
  {
    /* transfer bytes to the device */
    while ((write_count != data_length) &&
           (spi->CS.field.TXD != 0))
    {
      spi->FIFO = (uint32_t)*write_data;

      write_data++;
      write_count++;
    }

    /* drain the RX FIFO */
    while ((read_count != data_length) &&
           (spi->CS.field.RXD != 0))
    {
      if (read_data != NULL)
      {
        *read_data = spi->FIFO;

        read_data++;
        read_count++;
      }
      else
      {
        (void)spi->FIFO;

        read_count++;
      }
    }
  } while (spi->CS.field.DONE == 0);
  spi->CS.field.TA = 0;

  /* disable the chip select on the device */
  usleep( 10 );
  GPIO_SET( gpio, CS_pin );

  return;
}

void initialize_accelerometer_and_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union  LSM9DS1_transaction  transaction;

  /*
   * read the who_am_i register... just for kicks
   * in CTRL_REG_G, enable the gyro (low speed, low rate is fine)
   * in CTRL_REG_G, set the output to the data register
   * the status register has flags for whether data is available or not... I probably need to wait for boot complete
   * in CTRL_REG, enable the accel (low speed, low G is fine), BLE set to Little Endian (just in case)
   */
  /* print WHO_AM_I */
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_WHO_AM_I;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.WHO_AM_I), CS_AG_PIN, gpio, spi );
  printf( "WHOAMI (0x68) = 0x%2.2X\n", transaction.field.body.WHO_AM_I.SIX_EIGHT );

  /* in CTRL_REG1_G, enable the gyro (low speed, low rate is fine)
   * in CTRL_REG2_G, set the output to the data register
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG1_G;
  transaction.field.body.CTRL_REG1_G.BW_G         = 0;
  transaction.field.body.CTRL_REG1_G.FS_G         = 0;
  transaction.field.body.CTRL_REG1_G.ODR_G        = 1;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG1_G), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG2_G;
  transaction.field.body.CTRL_REG2_G.OUT_SEL      = 0;
  transaction.field.body.CTRL_REG2_G.INT_SEL      = 0;
  transaction.field.body.CTRL_REG2_G.zero         = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG2_G), CS_AG_PIN, gpio, spi );

  /* in CTRL_REG, enable the accel (low speed, low G is fine), set block data update (race conditions are bad), BLE set to Little Endian (just in case), disable I2C
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG4;
  transaction.field.body.CTRL_REG4.zero1          = 0;
  transaction.field.body.CTRL_REG4.Zen_G          = 1;
  transaction.field.body.CTRL_REG4.Yen_G          = 1;
  transaction.field.body.CTRL_REG4.Xen_G          = 1;
  transaction.field.body.CTRL_REG4.zero0          = 0;
  transaction.field.body.CTRL_REG4.LIR_XL1        = 0;
  transaction.field.body.CTRL_REG4.FOURD_XL1      = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG4), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG5_XL;
  transaction.field.body.CTRL_REG5_XL.DEC         = 0;
  transaction.field.body.CTRL_REG5_XL.Zen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.Yen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.Xen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.zero        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG5_XL), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG6_XL;
  transaction.field.body.CTRL_REG6_XL.ODR_XL      = 1;
  transaction.field.body.CTRL_REG6_XL.FS_XL       = 0;
  transaction.field.body.CTRL_REG6_XL.BW_SCAL_ODR = 0;
  transaction.field.body.CTRL_REG6_XL.BW_XL       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG6_XL), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG7_XL;
  transaction.field.body.CTRL_REG7_XL.HR          = 0;
  transaction.field.body.CTRL_REG7_XL.DCF         = 0;
  transaction.field.body.CTRL_REG7_XL.zero1       = 0;
  transaction.field.body.CTRL_REG7_XL.FDS         = 0;
  transaction.field.body.CTRL_REG7_XL.zero0       = 0;
  transaction.field.body.CTRL_REG7_XL.HPIS1       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG7_XL), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG8;
  transaction.field.body.CTRL_REG8.BOOT           = 0;
  transaction.field.body.CTRL_REG8.BDU            = 0;
  transaction.field.body.CTRL_REG8.H_LACTIVE      = 0;
  transaction.field.body.CTRL_REG8.PP_OD          = 0;
  transaction.field.body.CTRL_REG8.SIM            = 0;
  transaction.field.body.CTRL_REG8.IF_ADD_INC     = 1;  /* you still have to set the M_S bit in the command */
  transaction.field.body.CTRL_REG8.BLE            = 0;  /* ARM processors default to Little Endian */
  transaction.field.body.CTRL_REG8.SW_RESET       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG8), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG9;
  transaction.field.body.CTRL_REG9.zero1          = 0;
  transaction.field.body.CTRL_REG9.SLEEP_G        = 0;
  transaction.field.body.CTRL_REG9.zero0          = 0;
  transaction.field.body.CTRL_REG9.FIFO_TEMP_EN   = 0;
  transaction.field.body.CTRL_REG9.DRDY_mask_bit  = 0;
  transaction.field.body.CTRL_REG9.I2C_DISABLE    = 1;
  transaction.field.body.CTRL_REG9.FIFO_EN        = 0;
  transaction.field.body.CTRL_REG9.STOP_ON_FTH    = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG9), CS_AG_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG10;
  transaction.field.body.CTRL_REG10.zero1         = 0;
  transaction.field.body.CTRL_REG10.ST_G          = 0;
  transaction.field.body.CTRL_REG10.zero0         = 0;
  transaction.field.body.CTRL_REG10.ST_XL         = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG10), CS_AG_PIN, gpio, spi );

  return;
}

void initialize_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union  LSM9DS1_transaction  transaction;

  /* print WHO_AM_I */
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_WHO_AM_I_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.WHO_AM_I_M), CS_M_PIN, gpio, spi );
  printf( "WHOAMI_M (0x3D) = 0x%2.2X\n", transaction.field.body.WHO_AM_I_M.THREE_D);

  /*
   * in CTRL_REG1_M, no temperature compensation, enable X/Y, run in medium performance at 10Hz
   * in CTRL_REG2_M, use 4 gauss scale
   * in CTRL_REG3_M, run in continuous conversion, disable I2C, disable low-power (leave SIM default)
   * in CTRL_REG4_M, enable Z with Little Endian (just in case)
   * in CTRL_REG5_M, enable block data updates (race conditions are bad)
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG1_M;
  transaction.field.body.CTRL_REG1_M.TEMP_COMP    = 0;
  transaction.field.body.CTRL_REG1_M.OM           = 1;
  transaction.field.body.CTRL_REG1_M.DO           = 4;
  transaction.field.body.CTRL_REG1_M.ST           = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG1_M), CS_M_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG2_M;
  transaction.field.body.CTRL_REG2_M.zero2        = 0;
  transaction.field.body.CTRL_REG2_M.FS           = 0;
  transaction.field.body.CTRL_REG2_M.zero1        = 0;
  transaction.field.body.CTRL_REG2_M.REBOOT       = 0;
  transaction.field.body.CTRL_REG2_M.SOFT_RST     = 0;
  transaction.field.body.CTRL_REG2_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG2_M), CS_M_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG3_M;
  transaction.field.body.CTRL_REG3_M.I2C_DISABLE  = 1;
  transaction.field.body.CTRL_REG3_M.zero1        = 0;
  transaction.field.body.CTRL_REG3_M.LP           = 0;
  transaction.field.body.CTRL_REG3_M.zero0        = 0;
  transaction.field.body.CTRL_REG3_M.SIM          = 0;
  transaction.field.body.CTRL_REG3_M.MD           = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG3_M), CS_M_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG4_M;
  transaction.field.body.CTRL_REG4_M.zero1        = 0;
  transaction.field.body.CTRL_REG4_M.OMZ          = 1;
  transaction.field.body.CTRL_REG4_M.BLE          = 0;
  transaction.field.body.CTRL_REG4_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG4_M), CS_M_PIN, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG5_M;
  transaction.field.body.CTRL_REG5_M.zero1        = 0;
  transaction.field.body.CTRL_REG5_M.BDU          = 1;
  transaction.field.body.CTRL_REG5_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG5_M), CS_M_PIN, gpio, spi );

  return;
}

void read_accelerometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_XL_X;
  union uint16_to_2uint8      OUT_XL_Y;
  union uint16_to_2uint8      OUT_XL_Z;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 1000 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG), CS_AG_PIN, gpio, spi );
  } while (transaction.field.body.STATUS_REG.XLDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_X.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_X.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_Y.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_Y.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_Z.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_XL), CS_AG_PIN, gpio, spi );
  OUT_XL_Z.field.H                = transaction.value[1];

  acc[acci][0] = (float)OUT_XL_X.signed_value*2.0/32768.0*9.80665;
  acc[acci][1] = (float)OUT_XL_Y.signed_value*2.0/32768.0*9.80665;
  acc[acci][2] = (float)OUT_XL_Z.signed_value*2.0/32768.0*9.80665;

  if (acc[acci][0] > maxnum[0])
  	maxnum[0]=acc[acci][0];
  if (acc[acci][0] < minnum[0])
  	minnum[0]=acc[acci][0];
  if (acc[acci][1] > maxnum[1])
  	maxnum[1]=acc[acci][1];
  if (acc[acci][1] < minnum[1])
  	minnum[1]=acc[acci][1];
  if (acc[acci][2] > maxnum[2])
  	maxnum[2]=acc[acci][2];
  if (acc[acci][2] < minnum[2])
  	minnum[2]=acc[acci][2];

  //printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
      //acc[acci][0],  /* 2g range, 16-bit signed fixed-point */
     // acc[acci][1],
     // acc[acci][2] );

  acci++;
  return;
}

void read_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_X_G;
  union uint16_to_2uint8      OUT_Y_G;
  union uint16_to_2uint8      OUT_Z_G;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 1000 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG), CS_AG_PIN, gpio, spi );
  } while (transaction.field.body.STATUS_REG.GDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_XL), CS_AG_PIN, gpio, spi );
  OUT_X_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_XL), CS_AG_PIN, gpio, spi );
  OUT_X_G.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_XL), CS_AG_PIN, gpio, spi );
  OUT_Y_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_XL), CS_AG_PIN, gpio, spi );
  OUT_Y_G.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_XL), CS_AG_PIN, gpio, spi );
  OUT_Z_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_XL), CS_AG_PIN, gpio, spi );
  OUT_Z_G.field.H                = transaction.value[1];

  gy[gyi][0] = (float)OUT_X_G.signed_value*245.0/32768.0;
  gy[gyi][1] = (float)OUT_Y_G.signed_value*245.0/32768.0;
  gy[gyi][2] = (float)OUT_Z_G.signed_value*245.0/32768.0;

  if (gy[gyi][0] > maxnum[3])
  	maxnum[3]=gy[gyi][0];
  if (gy[gyi][0] < minnum[3])
  	minnum[3]=gy[gyi][0];
  if (gy[gyi][1] > maxnum[4])
  	maxnum[4]=gy[gyi][1];
  if (gy[gyi][1] < minnum[4])
  	minnum[4]=gy[gyi][1];
  if (gy[gyi][2] > maxnum[5])
  	maxnum[5]=gy[gyi][2];
  if (gy[gyi][2] < minnum[5])
  	minnum[5]=gy[gyi][2];

  //printf( "Gyro X: %.2f dps\ty=%.2f dps\tz=%.2f dps\n",
      //gy[gyi][0],  /* 245dps range, 16-bit signed fixed-point */
     // gy[gyi][1],
     // gy[gyi][2] );

  gyi++;
  return;
}

void read_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_X_M;
  union uint16_to_2uint8      OUT_Y_M;
  union uint16_to_2uint8      OUT_Z_M;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 1000 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG_M;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG_M), CS_M_PIN, gpio, spi );
  } while (transaction.field.body.STATUS_REG_M.ZYXDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_M), CS_M_PIN, gpio, spi );
  OUT_X_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_M), CS_M_PIN, gpio, spi );
  OUT_X_M.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_M), CS_M_PIN, gpio, spi );
  OUT_Y_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_M), CS_M_PIN, gpio, spi );
  OUT_Y_M.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_M), CS_M_PIN, gpio, spi );
  OUT_Z_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_M), CS_M_PIN, gpio, spi );
  OUT_Z_M.field.H                = transaction.value[1];

  mag[magi][0]=(float)OUT_X_M.signed_value*4.0/32768.0;
  mag[magi][1]=(float)OUT_Y_M.signed_value*4.0/32768.0;
  mag[magi][2]=(float)OUT_Z_M.signed_value*4.0/32768.0;

  if (mag[magi][0] > maxnum[6])
  	maxnum[6]=mag[magi][0];
  if (mag[magi][0] < minnum[6])
  	minnum[6]=mag[magi][0];
  if (mag[magi][1] > maxnum[7])
  	maxnum[7]=mag[magi][1];
  if (mag[magi][1] < minnum[7])
  	minnum[7]=mag[magi][1];
  if (mag[magi][2] > maxnum[8])
  	maxnum[8]=mag[magi][2];
  if (mag[magi][2] < minnum[8])
  	minnum[8]=mag[magi][2];

  //printf( "Mag X: %.2f gauss\ty=%.2f gauss\tz=%.2f gauss\n",
     // mag[magi][0],  /* 4 gauss range, 16-bit signed fixed-point */
    //  mag[magi][1],
     // mag[magi][2] );

  magi++;
  return;
}

void *readpara(void * arg){
	struct thread_parameter * parameter = (struct thread_parameter *)arg;
	//initialize
	initialize_accelerometer_and_gyroscope( (parameter->spi), (parameter->gpio) );
    initialize_magnetometer( (parameter->spi), (parameter->gpio) );
	while (!quit){
		if ((forw || back) && (readtime>acci)){
			read_accelerometer( parameter->spi, parameter->gpio );
			read_magnetometer( parameter->spi, parameter->gpio );
			read_gyroscope( parameter->spi, parameter->gpio );
		}
	}
}

void DimLevUnit(int Level, int pin, volatile struct gpio_register *gpio)
{
  int ONcount, OFFcount;

  ONcount = Level;
  OFFcount = 100 - Level;

  /* create the output pin signal duty cycle, same as Level */
  GPIO_SET( gpio, pin ); /* ON LED at GPIO 5 */
  while (ONcount > 0)
  {
    usleep( Tstep );
    ONcount = ONcount - 1;
  }
  GPIO_CLR( gpio, pin ); /* OFF LED at GPIO 5 */
  while (OFFcount > 0)
  {
    usleep( Tstep );
    OFFcount = OFFcount - 1;
  }
}

void *ThreadSW( void * arg )
{
  struct thread_parameter *parameter = (struct thread_parameter *) arg;
  pthread_mutex_lock(&(parameter->done->lock));
  while(!(parameter->done->done)){
  	pthread_mutex_unlock(&(parameter->done->lock));
  	//forward 10
	if (forw && !back && !sb){
		if (parameter->pin == 5){
			GPIO_SET( (parameter->gpio), 5);
		}
		if (parameter->pin == 6){
			GPIO_CLR( (parameter->gpio), 6);
		}
		if (parameter->pin == 22){
			GPIO_SET( (parameter->gpio), 22);
		}
		if (parameter->pin == 23){
			GPIO_CLR( (parameter->gpio), 23);
		}
	}
  	//backward 01 
	else if (!forw && back && !sb){
		if (parameter->pin == 5){
			GPIO_CLR( (parameter->gpio), 5);
		}
		if (parameter->pin == 6){
			GPIO_SET( (parameter->gpio), 6);
		}
		if (parameter->pin == 22){
			GPIO_CLR( (parameter->gpio), 22);
		}
		if (parameter->pin == 23){
			GPIO_SET( (parameter->gpio), 23);
		}
	}
  	//stop 00
	else if (!forw && !forw && !sb){
		if (parameter->pin == 5){
			GPIO_CLR( (parameter->gpio), 5);
		}
		if (parameter->pin == 6){
			GPIO_CLR( (parameter->gpio), 6);
		}
		if (parameter->pin == 22){
			GPIO_CLR( (parameter->gpio), 22);
		}
		if (parameter->pin == 23){
			GPIO_CLR( (parameter->gpio), 23);
		}
	}
  	//short break 11
  	else if (sb){
  		if (parameter->pin == 5){
			GPIO_SET( (parameter->gpio), 5);
		}
		if (parameter->pin == 6){
			GPIO_SET( (parameter->gpio), 6);
		}
		if (parameter->pin == 22){
			GPIO_SET( (parameter->gpio), 22);
		}
		if (parameter->pin == 23){
			GPIO_SET( (parameter->gpio), 23);
		}
	}
	pthread_mutex_lock(&(parameter->done->lock));
  }
  pthread_mutex_unlock(&(parameter->done->lock));

  return 0;
}

void *ThreadHW( void * arg )
{
  int                       iterations; /* used to limit the number of dimming iterations */
  int                       Timeu;      /* dimming repetition of each level */
  int                       DLevel;     /* dimming level as duty cycle, 0 to 100 percent */
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int                       speed = 100;
  int turnhigh = 70;
  int turnlow = 40;
  int m2speed = 50;

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    pthread_mutex_lock( &(parameter->pause->lock) );
    while (parameter->pause->pause)
    {
      pthread_mutex_unlock( &(parameter->pause->lock) );
      usleep( 10000 ); /* 10ms */
      pthread_mutex_lock( &(parameter->pause->lock) );
    }
    pthread_mutex_unlock( &(parameter->pause->lock) );

    if(mode == 2){
    	if(!needright && !needleft){
    		if (parameter->pin == 12){
	        parameter->pwm->DAT1 = m2speed;
	      	}
		    else if (parameter->pin == 13){
		    	parameter->pwm->DAT2 = m2speed;
	      	}
    	}
    	else if (needleft){
    		if (parameter->pin == 12){
		    		parameter->pwm->DAT1 = 0;
		    }
		    else if(parameter->pin == 13){
		    	parameter->pwm->DAT2 = m2speed;
		    }
		    printf("turnleft\n");
    	}
    	else if (needright){
    		if (parameter->pin == 12){
	    			parameter->pwm->DAT1 = m2speed;
	    	}
	    	else if(parameter->pin == 13){
	    		parameter->pwm->DAT2 = 0;
	    	}
		printf("turnrights\n");
    	}
    }

    //constant speed
    else if(mode == 1){
	    	//right
	    	while(right>0){
	    		if (parameter->pin == 12){
	    			parameter->pwm->DAT1 = turnhigh;
	    		}
	    		else{
	    			parameter->pwm->DAT2 = turnlow;
	    		}
	    	}

		//left
		while(left>0){
			if (parameter->pin == 12){
		    		parameter->pwm->DAT1 = turnlow;
		    	}
		    	else{
		    		parameter->pwm->DAT2 = turnhigh;
		    	}
	        }
	        //if initail condition: full speed
		    
	    //change speed
	    //inc
	    if(inc == 1){
	    	int subspeed = speed;
	    	while((subspeed<PWM_RANGE) && (subspeed < speed+10)){
	    		if(parameter->pin == 12){
	    			parameter->pwm->DAT1 = subspeed;
	        	}
	    		else if(parameter->pin == 13){
	    			parameter->pwm->DAT2 = subspeed;
	        	}
	    		usleep(Tlevel*Tstep*100);
	    		subspeed += 1;
	    	}
	    	speed = subspeed;
	      	//printf("increase speed to %i\n",speed);
	    	inc = 0;
	    }

	    //dec
	    else if(dec == 1){
	    	int subspeed = speed;
	    	while((subspeed>40) && (subspeed > speed-10)){
	    		if(parameter->pin == 12){
	    			parameter->pwm->DAT1 = subspeed;
	        	}
	    		else if(parameter->pin == 13){
	    			parameter->pwm->DAT2 = subspeed;
	        	}
	    		usleep(Tlevel*Tstep*100);
	    		subspeed -= 1;
	    	}
	    	speed = subspeed;
	    	dec = 0;
	      //printf("decrease speed to %i\n",speed);
	     }
	     if (init == 1){
		    	speed = 100;
		    }

		    //constant speed
		    if (parameter->pin == 12){
		      parameter->pwm->DAT1 = speed;
		      //printf("left:%d",parameter->pwm->DAT1);
	      	}
		    else{
		      parameter->pwm->DAT2 = speed;
		      //printf("right:%d",parameter->pwm->DAT2);
	      	}
	}
    pthread_mutex_lock( &(parameter->done->lock) );
  }
  pthread_mutex_unlock( &(parameter->done->lock) );

  return 0;
}



int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
}

void *ThreadKey( void * arg )
{
  struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  bool done;

  do
  {
    printf("hw5m%d>",mode);
    switch (get_pressed_key())
    {
      case 'q':
        done = true;
        quit = 1;
        /* unpause everything */
        pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
        thread_key_parameter->pause1->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
        thread_key_parameter->pause2->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
        thread_key_parameter->pause3->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
        thread_key_parameter->pause4->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
        thread_key_parameter->pause5->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
        thread_key_parameter->pause6->pause = false;
        pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
        forw = 0;
        back = 0;
        sb = 0;
        inc = 0;
        dec = 0;
        left = 0;
        right = 0;
        usleep(500000);
        /* indicate that it is time to shut down */
        pthread_mutex_lock( &(thread_key_parameter->done->lock) );
        thread_key_parameter->done->done = true;
        pthread_mutex_unlock( &(thread_key_parameter->done->lock) );
        break;

      //mode 
      case 'm':
      	printf("m");
      	switch (get_pressed_key()){
	      case '1':
		  mode = 1;
		  forw = 0;
		  back = 0;
		  sb = 0;
		  inc = 0;
		  dec = 0;
		  left = 0;
		  right = 0;
		  printf("1\n");
		  break;
      	  case '2':
		  mode = 2;
		  needright = 0;
		  needleft = 0;
		  forw = 0;
		  back = 0;
		  sb = 0;
		  printf("2\n");
		  break;

		case 'm':
		  printf("m\n");
		  float drawmap[120][2] = {};
		  int accten = floor(acci/10);
		  int gridmap[120][2] = {};
		  float minx =0;
		  float miny =0;
		  float maxx =0;
		  float maxy =0;
		  printf("accten:%d",accten);
		  if(acci==10*accten)
		  	accten--;
		  drawmap[0][0]= 0;
		  drawmap[0][1]= 0;
		  drawmap[accten+1][0] = coor[acci-1][0];
		  drawmap[accten+1][1] = coor[acci-1][1];

		  for (int i=1;i<=accten;i++){
		  	drawmap[i][0] = coor[i*10-1][0];
		  	drawmap[i][1] = coor[i*10-1][1];
		  	if(drawmap[i][0]>maxx)
		  		maxx=drawmap[i][0];
		  	if(drawmap[i][0]<minx)
		  		minx=drawmap[i][0];
		  	if(drawmap[i][1]>maxy)
		  		maxy=drawmap[i][1];
		  	if(drawmap[i][1]<miny)
		  		miny=drawmap[i][1];
		  }
		  if(drawmap[accten+1][0]>maxx)
		    maxx = drawmap[accten+1][0];
		  if(drawmap[accten+1][0]<minx)
		    minx = drawmap[accten+1][0];
		  if(drawmap[accten+1][1]>maxy)
		    maxy = drawmap[accten+1][1];
		  if(drawmap[accten+1][1]<miny)
		    miny = drawmap[accten+1][1];
		  for(int i=0; i<=accten+1; i++){
		    printf("drawmap:%.2f,%.2f\n",drawmap[i][0],drawmap[i][1]);
		  }
		  printf("minx:%.2f,maxx:%.2f,miny:%.2f,maxy:%.2f\n",minx,maxx,miny,maxy);
		  //grid
		  float blockx = (maxx-minx)/16;
		  float blocky = (maxy-miny)/16;
		  printf("blockx:%.2f,blocky:%.2f\n",blockx,blocky);
		  for (int i=0;i<=accten+1;i++){
		  	for (int j=1;j<=16;j++){
		  		if (drawmap[i][0]<=minx+blockx*j){
		  			gridmap[i][0]=j-1;
		  			break;
				}
		  	}
		  	for (int j=1;j<=16;j++){
		  		if (drawmap[i][1]<=miny+blocky*j){
		  			gridmap[i][1]=j-1;
					break;
				}
					
		  	}
		  }
		  for(int i=0; i<=accten+1;i++){
		    printf("gridmap:%d,%d\n",gridmap[i][0],gridmap[i][1]);
		  }
		  //speed
		  float speedmap[120] = {};
		  speedmap[0]=0;
		  speedmap[accten+1]= 0;
		  float maxspeed = 0;
		  for (int i=1;i<=accten;i++){
		  	float dissum = 0;
		  	for (int j=0;j<=9;j++){
		  		dissum += coor[(i-1)*10+j][2];
		  	}
		  	speedmap[i]=dissum;
		  	if (speedmap[i]>maxspeed)
		  		maxspeed = speedmap[i];
		  }
		  for(int i=0; i<=accten+1;i++){
		    printf("speedmap:%.2f\n",speedmap[i]);
		  }
		  
		  //speed level
		  int speedgrid[120]={};
		  float speedblock =  maxspeed/9;
		  printf("maxspeed:%.2f,speedblock:%.2f\n",maxspeed,speedblock);
		  for (int i=0;i<=accten+1;i++){
		  	for(int j=1;j<=9;j++){
		  		if(speedmap[i]<=speedblock*j){
		  			speedgrid[i]= j;
					break;
				      }
		  	}
		  }
		  speedgrid[0] = 0;
		  speedgrid[accten+1] = 0;
		  for(int i=0; i<=accten+1;i++){
		    printf("speedgrid:%d\n",speedgrid[i]);
		  }
		  //map array
		  for (int i=0; i<20;i++){
		  	for (int j=0; j<20;j++){
		  		if(i==0 || j==0 || i==19 || j==19){
		  			maparr[i][j]=0;
		  		}
		  		else{
		  			maparr[i][j]=-1;
		  		}
		  	}
		  }
		  //put pt in array
		  for (int i=0;i<=accten+1;i++){
		  	maparr[gridmap[i][0]+2][gridmap[i][1]+2] = speedgrid[i];
		  }
		  //print map
		  for (int i=0;i<20;i++){
		  	for(int j=0;j<20;j++){
		  		if(maparr[i][j]==-1)
		  			printf("   ");
		  		else{
		  			printf("%d  ",maparr[i][j]);
		  		}
		  	}
		  	printf("\n");
		  }
		  break;

		default:
		  break;
      	}
	  break;

      //pause
      case 's':
        pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
        thread_key_parameter->pause1->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
        thread_key_parameter->pause2->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
        thread_key_parameter->pause3->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
        thread_key_parameter->pause4->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
        thread_key_parameter->pause5->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
        pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
        thread_key_parameter->pause6->pause = true;
        pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
        forw = 0;
        back = 0;
        sb = 0;
        inc = 0;
        dec = 0;
        left = 0;
        right = 0;
        printf("s\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
        printf("readtime: %d, acccount: %d, gycount: %d, magcount: %d\n",readtime,acci,gyi,magi);
        break;

      //check readings
      case 'p':
        if (!forw && !back){
          printf("AC GY MG\n");
          //range
          for (int i =0; i<=8;i++){
            float diff = (maxnum[i] - minnum[i])/10;
            float base = minnum[i];
            for (int j=0; j<=8;j++){
              rangenum[i][j] = base+(j+1)*diff;
            }
          }
        for (int i=0;i<acci;i++){
          //read data
          float readline[9]={};
          int newline[9] = {};
          readline[0] = acc[i][0];
          readline[1] = acc[i][1];
          readline[2] = acc[i][2];
          readline[3] = gy[i][0];
          readline[4] = gy[i][1];
          readline[5] = gy[i][2];
          readline[6] = mag[i][0];
          readline[7] = mag[i][1];
          readline[8] = mag[i][2];
          for (int j = 0;j<=8;j++){
            if (readline[j]<rangenum[j][0])
              newline[j] = 0;
            else if (readline[j]<rangenum[j][1])
              newline[j] = 1;
            else if (readline[j]<rangenum[j][2])
              newline[j] = 2;
            else if (readline[j]<rangenum[j][3])
              newline[j] = 3;
            else if (readline[j]<rangenum[j][4])
              newline[j] = 4;
            else if (readline[j]<rangenum[j][5])
              newline[j] = 5;
            else if (readline[j]<rangenum[j][6])
              newline[j] = 6; 
            else if (readline[j]<rangenum[j][7])
              newline[j] = 7; 
            else if (readline[j]<rangenum[j][8])
              newline[j] = 8;
            else{
              newline[j] = 9;
            }
          }
          printf("%d%d%d%d%d%d%d%d%d\n",newline[0],newline[1],newline[2],newline[3],newline[4],newline[5],newline[6],newline[7],newline[8]);
        }
        printf("AC_x range: %.2f ~ %.2f, AC_y range: %.2f ~ %.2f, AC_z range: %.2f ~ %.2f\n",minnum[0],maxnum[0],minnum[1],maxnum[1],minnum[2],maxnum[2]);
        printf("GY_x range: %.2f ~ %.2f, GY_y range: %.2f ~ %.2f, GY_z range: %.2f ~ %.2f\n",minnum[3],maxnum[3],minnum[4],maxnum[4],minnum[5],maxnum[5]);
        printf("MG_x range: %.2f ~ %.2f, MG_y range: %.2f ~ %.2f, MG_z range: %.2f ~ %.2f\n",minnum[6],maxnum[6],minnum[7],maxnum[7],minnum[8],maxnum[8]);
        }
	break;

      //speed
      case 'n':{
	float acp[1200][2] ={};
      	//low pass filter
      	for (int i=1; i<acci-1; i++){
      		acp[i][0] = acc[i-1][0]/5+acc[i][0]*3/5+acc[i+1][0]/5;
      		acp[i][1] = acc[i-1][1]/5+acc[i][1]*3/5+acc[i+1][1]/5;
      	}
      	acp[0][0] = acc[0][0]*3/5+acc[1][0]/5;
      	acp[0][1] = acc[0][1]*3/5+acc[1][1]/5;
      	acp[acci-1][0] = acc[acci-1][0]*3/5+acc[acci-2][0]/5;
      	acp[acci-1][1] = acc[acci-1][1]*3/5+acc[acci-2][1]/5;
      	//start getting data
        if (!forw&&!back){
          float speednumx = 0;
          float speednumy = 0;
          float totaldist = 0;
          float coorx =0;
          float coory =0;
          for(int i=0; i<acci;i++){
            speednumx+=acp[i][0]*0.1;
            speednumy+=acp[i][1]*0.1;
            coorx += speednumx*0.1;
            coory += speednumy*0.1;
            coor[i][0] = coorx;
            coor[i][1] = coory;
            coor[i][2] = sqrtf(speednumx*speednumx*0.01+speednumy*speednumy*0.01);
            totaldist += coor[i][2];
	    	printf("a:%.2f,%.2f; smooth:%.2f,%.2f; s:%.2f,%.2f; coor:%.2f,%.2f; dist:%.2f\n",acc[i][0],acc[i][1],acp[i][0],acp[i][1],speednumx,speednumy,coor[i][0],coor[i][1],coor[i][2]);
          }
          printf("Total distance is %.2fm, average speed is %.2fm/s\n",totaldist,totaldist/(acci*0.1));
        }
      }
	break;
        /*
for(int i=0; i<acci,i++){
            float diff = speedmax/10;
            for (int j=0; j<10; j++){
              if (speedarr[i][2]<=diff*(1+j)){
                speedarr[i][3]= (float)j;
                break;
              }
            }
          }


*/
      //forward
      case 'f':
      	if (mode==1){
	        if (!forw && !back){
	          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	          thread_key_parameter->pause2->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
	          thread_key_parameter->pause3->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
	          thread_key_parameter->pause4->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
	          thread_key_parameter->pause5->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
	          thread_key_parameter->pause6->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
	          acci = 0;
	          gyi = 0;
	          magi = 0;
            readtime = 1;
	          memset(maxnum,0,9);
	          memset(minnum,0,9);
	        }
	      	//previous not go forward
	      	if(forw!=1){
	      		init = 1;
	      	}
	      	inc = 0;
	      	dec = 0;
	      	left = 0;
	      	right = 0;
	      	//if was going backward
	      	if (back==1){
	      		pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = true;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = true;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 1;
	        	usleep(500000);
	        	pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	        	thread_key_parameter->pause1->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	        	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 0;
	      	}
	      	forw = 1;
	      	back = 0;
	      	printf("forward\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	}
      	else if(mode == 2){
      		if (!forw){
      		  forw = 1;
      		  back = 0;
      		  pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	          thread_key_parameter->pause2->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
	          thread_key_parameter->pause3->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
	          thread_key_parameter->pause4->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
	          thread_key_parameter->pause5->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
	          thread_key_parameter->pause6->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
	          acci = 0;
	          gyi = 0;
	          magi = 0;
            readtime = 1;
	          memset(maxnum,0,9);
	          memset(minnum,0,9);
	        }
      	}
      	break;


      //forward
      case 'w':
      	if (mode == 1){
	        if (!forw && !back){
	          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	          thread_key_parameter->pause2->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
	          thread_key_parameter->pause3->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
	          thread_key_parameter->pause4->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
	          thread_key_parameter->pause5->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
	          thread_key_parameter->pause6->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
	          acci = 0;
	          gyi = 0;
	          magi = 0;
            readtime = 1;
	          memset(maxnum,0,9);
	          memset(minnum,0,9);
	        }
	      	//previous not go forward
	      	if(forw!=1){
	      		init = 1;
	      	}
	      	inc = 0;
	      	dec = 0;
	      	left = 0;
	      	right = 0;
	      	//if going backward
	      	if (back==1){
	      		pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = true;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = true;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 1;
	        	usleep(500000);
	        	pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	        	thread_key_parameter->pause1->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	        	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 0;
	      	}
	      	forw = 1;
	      	back = 0;
	      	printf("forward\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
	      }
	      break;

      //backward
      case 'b':
      	if (mode == 1){
	        if (!forw && !back){
	          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	          thread_key_parameter->pause2->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
	          thread_key_parameter->pause3->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
	          thread_key_parameter->pause4->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
	          thread_key_parameter->pause5->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
	          thread_key_parameter->pause6->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
	          acci = 0;
	          gyi = 0;
	          magi = 0;
            readtime = 1;
	          memset(maxnum,0,9);
	          memset(minnum,0,9);
	        }
	        if(back!=1){
		  init = 1;
	        }
	      	inc = 0;
	      	dec = 0;
	      	right = 0;
	      	left = 0;
	      	//if going forward
	      	if (forw==1){
	      		pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          	thread_key_parameter->pause1->pause = true;
	          	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = true;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 1;
	        	usleep(500000);
	        	pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	        	thread_key_parameter->pause1->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	        	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 0;
	      	}
	      	forw = 0;
	      	back = 1;
	      	printf("backward\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right)
	      }
	      break;


      //backward
      case 'x':
      	if (mode == 1){
	        if (!forw && !back){
	          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          thread_key_parameter->pause1->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	          thread_key_parameter->pause2->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
	          thread_key_parameter->pause3->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
	          thread_key_parameter->pause4->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause5->lock) );
	          thread_key_parameter->pause5->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause5->lock) );
	          pthread_mutex_lock( &(thread_key_parameter->pause6->lock) );
	          thread_key_parameter->pause6->pause = false;
	          pthread_mutex_unlock( &(thread_key_parameter->pause6->lock) );
	          acci = 0;
	          gyi = 0;
	          magi = 0;
            readtime = 1;
	          memset(maxnum,0,9);
	          memset(minnum,0,9);
	        }
	        if(back!=1){
	        	init = 1;
	        }
	      	inc = 0;
	      	dec = 0;
	      	right = 0;
	      	left = 0;
	      	//if going forward
	      	if (forw==1){
	      		pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	          	thread_key_parameter->pause1->pause = true;
	          	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	          	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = true;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 1;
	        	usleep(500000);
	        	pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
	        	thread_key_parameter->pause1->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
	        	pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
	        	thread_key_parameter->pause2->pause = false;
	        	pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
	        	sb = 0;
	      	}
	      	forw = 0;
	      	back = 1;
	      	printf("backward\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
	      }
	      break;


      //faseter
      case 'i':
      	if(mode!=1)
      		break;
      	init = 0;
      	inc = 1;
      	dec = 0;
      	printf("faster\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      //slower
      case 'j':
      	if(mode!=1)
      		break;
      	init = 0;
      	inc = 0;
      	dec = 1;
      	printf("slower\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      //left	
      case 'l':
      	if(mode!=1)
      		break;
      	inc = 0;
      	dec = 0;
      	left++;
      	printf("left\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      //left	
      case 'a':
      	if(mode!=1)
      		break;
      	inc = 0;
      	dec = 0;
      	left++;
      	printf("left\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      //right	
      case 'r':
      	if(mode!=1)
      		break;
      	inc = 0;
      	dec = 0;
      	right++;
      	printf("right\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      //right	
      case 'd':
      	if(mode!=1)
      		break;
      	inc = 0;
      	dec = 0;
      	right++;
      	printf("right\n");//:init:%i,forw:%i,back:%i,sb:%i,inc:%i,dec:%i,left:%i,right:%i\n",init,forw,back,sb,inc,dec,left,right);
      	break;

      default:
        break;
    }
  } while (!done);
  printf( "key thread exiting\n" );

  return (void *)0;
}

void *counttime(void *t){
  while (!quit){
    if (forw || back){
      usleep(100000);
      readtime++;
    }
  }
}


void *turntime(void *t){
	while (!quit){
		if(left||right){
			usleep(100000);
		if(left){
			left--;
		}
		if(right){
			right--;
		}
		}
	}
}

void *sensor( void * arg ){
	struct thread_parameter * parameter = (struct thread_parameter *)arg;
	while(!quit){
		if(parameter->pin == 25){
			if(GPIO_READ(parameter->gpio,25) == 0){
				needleft = 0;
			}
			else{
				needleft = 1;
			}
		}
		else{
			if(GPIO_READ(parameter->gpio,26) == 0){
				needright = 0;
			}
			else{
				needright = 1;
			}
		}
	}
}

int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread12_handle;
  pthread_t                       thread13_handle;
  pthread_t                       thread5_handle;
  pthread_t                       thread6_handle;
  pthread_t                       thread22_handle;
  pthread_t                       thread23_handle;
  pthread_t                       thread25_handle;
  pthread_t                       thread26_handle;
  pthread_t                       thread_key_handle;
  pthread_t                       turntiming;
  pthread_t                       reading;
  pthread_t                       timing;

  struct done_flag                done   = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause1 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause2 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause3 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause4 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause5 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause6 = {PTHREAD_MUTEX_INITIALIZER, false};

  struct thread_parameter         thread12_parameter;
  struct thread_parameter         thread13_parameter;
  struct thread_parameter         thread5_parameter;
  struct thread_parameter         thread6_parameter;
  struct thread_parameter         thread22_parameter;
  struct thread_parameter         thread23_parameter;
  struct thread_parameter         thread25_parameter;
  struct thread_parameter         thread26_parameter;
  struct key_thread_parameter     thread_key_parameter;
  struct thread_parameter         reading_parameter;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );
    printf( "press m1 for regular mode: \n");
    printf( "press f/w to move forward at full speed \n");
    printf("press b/x to move backward at full speed\n");
    printf("duplicate forward/backward would be ignored\n");
    printf("press s to stop/pause\n");
    printf("press i/j to increase/decrease the speed by 10%\n");
    printf("press r/a to turn right 15 degree\n");
    printf("press l/d to turn left 15 degree\n");
    printf( "press m2 for tracking mode: \n");
    printf( "press f to move forward with line tracing \n");
    printf( "press s to stop \n");
    printf("for general operation:\n");
    printf("press p to check the accelerometer,gyroscope,magnetometer readings\n");
    printf("press n to check the average speed and distance for the last drive\n");
    printf("press mm to check the map of travel path\n");
    printf("press q to quit the program\n");

    enable_pwm_clock( io );

    /* set the pin function to alternate function 0 for GPIO12 */
    /* set the pin function to alternate function 0 for GPIO13 */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL6 = GPFSEL_INPUT;

    /* set the pin function to alternate function 0 for GPIO09 (SPI, MISO) */
    /* set the pin function to alternate function 0 for GPIO10 (SPI, MOSI) */
    /* set the pin function to alternate function 0 for GPIO11 (SPI, SCK) */
    /* set the pin function to output for GPIO18 (DEN_AG) */
    /* set the pin function to output for GPIO19 (CS_M) */
    /* set the pin function to output for GPIO20 (CS_AG) */
    io->gpio.GPFSEL0.field.FSEL9 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL0 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL1 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL8 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL9 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL0 = GPFSEL_OUTPUT;

    /* set initial output state */
    GPIO_SET(&(io->gpio), DEN_PIN);
    GPIO_SET(&(io->gpio), CS_M_PIN);
    GPIO_SET(&(io->gpio), CS_AG_PIN);
    usleep( 100000 );

    /* set up the SPI parameters */
    io->spi.CLK.field.CDIV = ((ROUND_DIVISION(250000000,4000000))>>1)<<1; /* this number must be even, so shift the LSb into oblivion */
    io->spi.CS.field.CS       = 0;
    io->spi.CS.field.CPHA     = 1;  /* clock needs to idle high and clock in data on the rising edge */
    io->spi.CS.field.CPOL     = 1;
    io->spi.CS.field.CLEAR    = 0;
    io->spi.CS.field.CSPOL    = 0;
    io->spi.CS.field.TA       = 0;
    io->spi.CS.field.DMAEN    = 0;
    io->spi.CS.field.INTD     = 0;
    io->spi.CS.field.INTR     = 0;
    io->spi.CS.field.ADCS     = 0;
    io->spi.CS.field.REN      = 0;
    io->spi.CS.field.LEN      = 0;
    /* io->spi.CS.field.LMONO */
    /* io->spi.CS.field.TE_EN */
    /* io->spi.CS.field.DONE */
    /* io->spi.CS.field.RXD */
    /* io->spi.CS.field.TXD */
    /* io->spi.CS.field.RXR */
    /* io->spi.CS.field.RXF */
    io->spi.CS.field.CSPOL0   = 0;
    io->spi.CS.field.CSPOL1   = 0;
    io->spi.CS.field.CSPOL2   = 0;
    io->spi.CS.field.DMA_LEN  = 0;
    io->spi.CS.field.LEN_LONG = 0;

    /* configure the PWM channels */
    io->pwm.RNG1 = PWM_RANGE;     /* the default value */
    io->pwm.RNG2 = PWM_RANGE;     /* the default value */
    io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    thread12_parameter.pin = 12;
    thread12_parameter.gpio = &(io->gpio);
    thread12_parameter.pwm = &(io->pwm);
    thread12_parameter.done = &done;
    thread12_parameter.pause = &pause1;
    thread13_parameter.pin = 13;
    thread13_parameter.pwm = &(io->pwm);
    thread13_parameter.gpio = &(io->gpio);
    thread13_parameter.done = &done;
    thread13_parameter.pause = &pause2;
    thread5_parameter.pin = 5;
    thread5_parameter.pwm = &(io->pwm);
    thread5_parameter.gpio = &(io->gpio);
    thread5_parameter.done = &done;
    thread5_parameter.pause = &pause3;
    thread6_parameter.pin = 6;
    thread6_parameter.pwm = &(io->pwm);
    thread6_parameter.gpio = &(io->gpio);
    thread6_parameter.done = &done;
    thread6_parameter.pause = &pause4;
    thread22_parameter.pin = 22;
    thread22_parameter.gpio = &(io->gpio);
    thread22_parameter.pwm = &(io->pwm);
    thread22_parameter.done = &done;
    thread22_parameter.pause = &pause5;
    thread23_parameter.pin = 23;
    thread23_parameter.pwm = &(io->pwm);
    thread23_parameter.gpio = &(io->gpio);
    thread23_parameter.done = &done;
    thread23_parameter.pause = &pause6;
    thread25_parameter.pin = 25;
    thread25_parameter.gpio = &(io->gpio);
    thread25_parameter.pwm = &(io->pwm);
    thread26_parameter.pin = 26;
    thread26_parameter.pwm = &(io->pwm);
    thread26_parameter.gpio = &(io->gpio);

    reading_parameter.spi = &(io->spi);
    reading_parameter.gpio = &(io->gpio);

    thread_key_parameter.done = &done;
    thread_key_parameter.pause1 = &pause1;
    thread_key_parameter.pause2 = &pause2;
    thread_key_parameter.pause3 = &pause3;
    thread_key_parameter.pause4 = &pause4;
    thread_key_parameter.pause5 = &pause5;
    thread_key_parameter.pause6 = &pause6;

    pthread_create( &turntiming, 0, turntime, NULL);
    pthread_create( &thread12_handle, 0, ThreadHW, (void *)&thread12_parameter );
    pthread_create( &thread13_handle, 0, ThreadHW, (void *)&thread13_parameter );
    pthread_create( &thread5_handle, 0, ThreadSW, (void *)&thread5_parameter );
    pthread_create( &thread6_handle, 0, ThreadSW, (void *)&thread6_parameter );
    pthread_create( &thread22_handle, 0, ThreadSW, (void *)&thread22_parameter );
    pthread_create( &thread23_handle, 0, ThreadSW, (void *)&thread23_parameter );
    pthread_create( &thread25_handle, 0, sensor, (void *)&thread25_parameter );
    pthread_create( &thread26_handle, 0, sensor, (void *)&thread26_parameter );
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    pthread_create( &timing, 0, counttime, NULL);
    pthread_create( &reading, 0, readpara, (void *)&reading_parameter);

    pthread_join( timing, 0);
    pthread_join( turntiming, 0);
    pthread_join( thread12_handle, 0 );	
    pthread_join( thread13_handle, 0 );
    pthread_join( thread5_handle, 0 );
    pthread_join( thread6_handle, 0 );
    pthread_join( thread22_handle, 0 );
	 pthread_join( thread23_handle, 0 );
	 pthread_join( thread25_handle, 0 );
	 pthread_join( thread26_handle, 0 );
    pthread_join( thread_key_handle, 0 );
  }
  printf("quiting...");
  return 0;
}

