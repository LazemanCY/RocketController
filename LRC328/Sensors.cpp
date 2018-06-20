#include "Arduino.h"
#include "config.h"
#include "Sensors.h"

#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

uint16_t i2c_errors_count = 0;

extern uint8_t rawADC[6];

//---------------------------I2C Functions---------------------------
void i2c_init(void) {
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / 400000) - 16) / 2;          // set the I2C clock rate to 400kHz
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
  i2c_errors_count = 0;
}

void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr) {
  TWCR = twcr;
  uint8_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      #if defined(WMP)
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      #endif
      i2c_errors_count++;
      break;
    }
  }
}

void i2c_rep_start(uint8_t address) {
  waitTransmissionI2C((1<<TWINT) | (1<<TWSTA) | (1<<TWEN)); // send REPEAT START condition and wait until transmission completed
  TWDR = address;                                           // send device address
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));              // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));
}

uint8_t i2c_readAck() {
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN) | (1<<TWEA));
  return TWDR;
}

uint8_t i2c_readNak() {
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));
  uint8_t r = TWDR;
  i2c_stop();
  return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  uint8_t *b = buf;
  while (--size) *b++ = i2c_readAck(); // acknowledge all but the final byte
  *b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, rawADC, 6);
}


void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}
//---------------------------END of I2C Functions---------------------------

//---------------------------MPU6050 Functions---------------------------
static void Gyro_init() {
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, GYRO_DLPF_CFG);    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  //GYRO_Common();
}

static void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}

void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  //ACC_Common();
}

void initS() {
  i2c_init();
  Gyro_init();
  //Baro_init();
  //Mag_init();
  ACC_init();
}

void initSensors() {
  uint8_t c = 5;
  while(c) { // We try several times to init all sensors without any i2c errors. An I2C error at this stage might results in a wrong sensor settings
    c--;
    initS();
    if (i2c_errors_count == 0) break; // no error during init => init ok
  }
}
