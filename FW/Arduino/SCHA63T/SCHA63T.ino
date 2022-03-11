/*
 *  Murata SCHA63T-K03 IMU I/F Board with Seeeduino XIAO
 * 
 *  References:
 *  [1] Murata, SCHA63T-K03: 6-DOF XYZ-Axis Gyroscope and xyz-Axis Accelerometer
 *      with digital SPI interface - Data Sheet, 2021 (https://www.murata.com)
 *  [2] Platform/Seeeduino XIAO (https://wiki.seeedstudio.com/Seeeduino-XIAO)
 *  [3] Arduino Language Reference (https://www.arduino.cc/reference)
 *
 *  Author:
 *  T.TAKASU
 *
 *  Histroy:
 *  2021-08-21  1.0.0  First version. 
 * 
 */

#include <SPI.h>
#include <TimerTC3.h> // library for Seeeduino XIAO

// -----------------------------------------------------------------------------

#define DEV_NAME     "SCHA63T-K03" // device name
#define FW_VER       "1.0.0"  // F/W version

#define CS_DUE       0        // Seeeduino XIAO PIN 1: (O) SPI CS DUE
#define CS_UNO       1        // Seeeduino XIAO PIN 2: (O) SPI CS UNO
#define PPS          2        // Seeeduino XIAO PIN 3: (I) PPS
#define LED_PPS      3        // Seeeduino XIAO PIN 4: (O) PPS LED
#define TEST_PIN     4        // Seeeduino XIAO PIN 5: (O) TEST PIN
#define LED_T        11       // Seeeduino XIAO LED T: (O) (blue)
#define LED_R        12       // Seeeduino XIAO LED R: (O) (blue)
#define LED_L        13       // Seeeduino XIAO LED L: (O) (orange)

#define REG_DUE_RZ   0x01     // SCHA63T register DUE Rate Z
#define REG_DUE_RY   0x03     // SCHA63T register UNO Rate Y
#define REG_DUE_RZ2  0x0B     // SCHA63T register UNO Rate Z2
#define REG_DUE_RY2  0x0D     // SCHA63T register UNO Rate Y2
#define REG_UNO_RX   0x01     // SCHA63T register DUE Rate X
#define REG_UNO_AX   0x04     // SCHA63T register DUE Acc X
#define REG_UNO_AY   0x05     // SCHA63T register DUE Acc Y
#define REG_UNO_AZ   0x06     // SCHA63T register DUE Acc Z
#define REG_UNO_RX2  0x0B     // SCHA63T register DUE Rate X2
#define REG_TEMP     0x07     // SCHA63T register temperature
#define REG_SUMSTAT  0x0E     // SCHA63T register summary status
#define REG_FILT_G   0x16     // SCHA63T register gyro filter control
#define REG_SYS_TEST 0x17     // SCHA63T register SYS_TEST
#define REG_RESET    0x18     // SCHA63T register reset control
#define REG_MODE     0x19     // SCHA63T register mode control
#define REG_FILT_A   0x1A     // SCHA63T register ACC filter control
#define REG_CID      0x1B     // SCHA63T register component ID
#define REG_TR2      0x1C     // SCHA63T register traceability 2
#define REG_TR0      0x1D     // SCHA63T register traceability 0
#define REG_TR1      0x1E     // SCHA63T register traceability 1
#define REG_SELBNK   0x1F     // SCHA63T register select bank
#define SCLK_RATE    10000000 // SCHA63T SPI SCLK rate (Hz)

#define SCALE_GYRO (1.0/80)   // SCHA63T gyro scale (deg/s/LSB) (nominal)
#define SCALE_ACCL (1.0/4905) // SCHA63T accelerometer scale (g/LSB) (nominal)
#define SCALE_TEMP (1.0/30)   // SCHA63T temperature scale (C/LSB)

#define HOST_RATE    115200   // HOST bitrate (bps)
#define NMEA_RATE    9600     // NMEA bitrate (bps)
#define PPS_MODE     RISING   // PPS interface mode (RISING or FALLING)
#define SAMP_RATE    200      // sampling rate (Hz)
#define MAX_RATE     200      // max data output rate (Hz)
#define MAX_TRY_INIT 3        // max try of device initializtion

#define LED_INTV     1000     // LED L blink interval (ms)
#define LED_DUTY     5        // LED blink duty-ratio

#define BYTE_L(x)    ((x) & 0xFF)
#define BYTE_H(x)    (((x) >> 8) & 0xFF)
#define ROUND(x)     (long)((x) + 0.5)

typedef void command_func_t(const char *); // command function type

// -----------------------------------------------------------------------------

volatile uint16_t data_rate;  // data output rate (Hz)
volatile double samp_intv;    // sampling interval (us)
volatile uint8_t samp_enable; // sampling enable (0:disable,1:enable)
volatile uint8_t time_sync;   // external time sync status (0:async,1:PPS,2:NMEA)
volatile uint8_t data_ready;  // data ready status (0:no,1:ready)
volatile uint32_t time_ms;    // data time (ms) (TOD in UTC: time_sync=NMEA)
volatile double data_gyro[3]; // data buffer gyro  {Rx, Ry, Rz} (deg/s)
volatile double data_accl[3]; // data buffer accel {Ax, Ay, Az} (g)
volatile uint16_t samp_count; // sample count in data buffer
volatile uint32_t pps_tick;   // tick of PPS signal
volatile uint8_t rang_sel[] = {0, 0}; // dynamic range selection gyro/accel
volatile uint8_t filt_sel[] = {2, 2}; // filter selection gyro/accel

volatile uint8_t debug_level = 0; // debug trace level (0:no debug)

char buff_command[128];       // command buffer
int p_command = 0;            // command buffer pointer
char buff_nmea[128];          // NMEA sentense buffer
int p_nmea = 0;               // NMEA sentense buffer pointer

// cross-axis compensation table ([1] 2.6)
float Cx[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // gyro
float Bx[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // accel

// STARTUP routine -------------------------------------------------------------
void setup() {  
  pinMode(CS_DUE, OUTPUT);
  pinMode(CS_UNO, OUTPUT);
  pinMode(PPS, INPUT_PULLUP);
  pinMode(TEST_PIN, OUTPUT);
  pinMode(LED_PPS, OUTPUT);
  
  digitalWrite(CS_DUE, HIGH);
  digitalWrite(CS_UNO, HIGH);
  digitalWrite(TEST_PIN, LOW);
  digitalWrite(LED_L, LOW);
  digitalWrite(LED_PPS, LOW);
  
  // start HOST serials
  Serial.begin(HOST_RATE);
  delay(100);
  
  // start SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(SCLK_RATE, MSBFIRST, SPI_MODE0));
  delay(100);
  
  // initialize device
  if (!init_device()) {
    init_err();
  }
  // initialize global variables
  init_vars();
  
  // start interval timer
  TimerTc3.initialize(ROUND(samp_intv));
  TimerTc3.attachInterrupt(timer_ISR);
  delay(100);
  
  // start PPS interrupts
  attachInterrupt(digitalPinToInterrupt(PPS), pps_ISR, PPS_MODE);
  delay(100);
  
  // start NMEA serials
  Serial1.begin(NMEA_RATE);
  delay(100);
  
  // enable sampling
  samp_enable = 1;
  
  digitalWrite(LED_L, HIGH);
}

// MAIN loop -------------------------------------------------------------------
void loop() {
  read_command();
  
  if (data_ready) {
    double gyro[3], accl[3], temp[2];
    uint8_t stat = read_stat(temp);
    
    noInterrupts();
    uint8_t t_sync = time_sync;
    uint32_t t_ms = time_ms;
    for (int i = 0; i < 3; i++) {
      gyro[i] = data_gyro[i];
      accl[i] = data_accl[i];
    }
    data_ready = samp_count = 0;
    interrupts();
    
    write_data(t_sync, t_ms, gyro, accl, temp, stat);
  }
  read_nmea();
}

// initialize SCHA63T ([1] Figure 7) -------------------------------------------
bool init_device() {
  trace(3, "init_device\r\n");
  
  // reset UNO/DUE
  write_reg(CS_UNO, REG_RESET, 1);
  write_reg(CS_DUE, REG_RESET, 1);
  delay(25);
  
  // read cross-axis compensation table
  read_axis_table();
  
  for (int i = 0; i < MAX_TRY_INIT; i++) {
    // set operation mode on
    write_reg(CS_UNO, REG_MODE, 0);
    write_reg(CS_DUE, REG_MODE, 0);
    write_reg(CS_DUE, REG_MODE, 0);
    delay(70);
    
    // set sensor dynamic ranges and filters
    set_rang_filt();
    
    // set EOI = 1
    write_reg(CS_UNO, REG_RESET, 2);
    write_reg(CS_DUE, REG_RESET, 2);
    
    if (test_init_end(CS_UNO) && test_init_end(CS_DUE)) {
      return true;
    }
    write_reg(CS_UNO, REG_RESET, 1); // reset UNO/DUE
    write_reg(CS_DUE, REG_RESET, 1);
    delay(25);
  }
  trace(2, "reset_device error\r\n");
  return false;
}

// read cross-axis compensation table ([1] 2.6) --------------------------------
void read_axis_table() {
  static const uint8_t reg_axis[] = // cross-axis compensation registers
    {0x0B, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x1B, 0x1C};
  uint8_t reg_mode = REG_MODE;
  uint16_t data1[1], data2[9];
  
  trace(3, "read_axis_table\r\n");
  
  // enter test mode
  write_reg(CS_DUE, reg_mode, 0x10); // unlock code 010
  write_reg(CS_DUE, reg_mode, 0x08); // unlock code 001
  write_reg(CS_DUE, reg_mode, 0x20); // unlock code 100
  if (!read_reg(CS_DUE, &reg_mode, 1, data1) || (data1[0] & 0x7) != 0x7) {
    trace(2, "test mode enter error\r\n");
    return;
  }
  write_reg(CS_DUE, REG_SELBNK, 5); // change bank to 5
  
  if (read_reg(CS_DUE, reg_axis, 9, data2)) {
    Cx[0][0] = (int8_t)BYTE_L(data2[0]) / 4096.0f + 1.0f; // Cxx
    Cx[0][1] = (int8_t)BYTE_H(data2[0]) / 4096.0f;        // Cxy
    Cx[0][2] = (int8_t)BYTE_L(data2[1]) / 4096.0f;        // Cxz
    Cx[1][0] = (int8_t)BYTE_H(data2[1]) / 4096.0f;        // Cyx
    Cx[1][1] = (int8_t)BYTE_L(data2[2]) / 4096.0f + 1.0f; // Cyy
    Cx[1][2] = (int8_t)BYTE_H(data2[2]) / 4096.0f;        // Cyz
    Cx[2][0] = (int8_t)BYTE_L(data2[3]) / 4096.0f;        // Czx
    Cx[2][1] = (int8_t)BYTE_H(data2[3]) / 4096.0f;        // Czy
    Cx[2][2] = (int8_t)BYTE_L(data2[4]) / 4096.0f + 1.0f; // Czz
    Bx[0][0] = (int8_t)BYTE_H(data2[4]) / 4096.0f + 1.0f; // bxx
    Bx[0][1] = (int8_t)BYTE_L(data2[5]) / 4096.0f;        // bxy
    Bx[0][2] = (int8_t)BYTE_H(data2[5]) / 4096.0f;        // bxz
    Bx[1][0] = (int8_t)BYTE_L(data2[6]) / 4096.0f;        // byx
    Bx[1][1] = (int8_t)BYTE_H(data2[6]) / 4096.0f + 1.0f; // byy
    Bx[1][2] = (int8_t)BYTE_L(data2[7]) / 4096.0f;        // byz
    Bx[2][0] = (int8_t)BYTE_H(data2[7]) / 4096.0f;        // bzx
    Bx[2][1] = (int8_t)BYTE_L(data2[8]) / 4096.0f;        // bzy
    Bx[2][2] = (int8_t)BYTE_H(data2[8]) / 4096.0f + 1.0f; // bzz
    
    trace(3,"axis_reg=%04X %04X %04X %04X %04X %04X %04X %04X %04X\r\n",
      data2[0], data2[1], data2[2], data2[3], data2[4], data2[5], data2[6],
      data2[7], data2[8]);
  }
  write_reg(CS_DUE, REG_SELBNK, 0); // restore bank to 0
  write_reg(CS_DUE, REG_RESET, 1);  // reset DUE
  delay(25);
}

// set sensor dynamic ranges and filters ---------------------------------------
void set_rang_filt() {
  write_reg(CS_DUE, REG_FILT_G, // DUE: Rz2_DYN,Rz2_FILT,Ry2_DYN,Ry2_FILT
            ((uint16_t)rang_sel[0] << 14) | ((uint16_t)filt_sel[0] << 11) |
            ((uint16_t)rang_sel[0] <<  6) | ((uint16_t)filt_sel[0] <<  3));
  write_reg(CS_UNO, REG_FILT_G, // UNO: Rx2_DYN,Rx2_FILT
            ((uint16_t)rang_sel[0] << 14) | ((uint16_t)filt_sel[0] << 11));
  write_reg(CS_UNO, REG_FILT_A, // UNO: Ax_DYN,Ax_FILT,Ay_DYN,Ay_FILT,Az_DYN,Az_FILT
            ((uint16_t)rang_sel[1] << 11) | ((uint16_t)filt_sel[1] << 8) |
            ((uint16_t)rang_sel[1] <<  7) | ((uint16_t)filt_sel[1] << 4) |
            ((uint16_t)rang_sel[1] <<  3) | ((uint16_t)filt_sel[1] << 0));
  delay(525);
}

// test initialization completed -----------------------------------------------
bool test_init_end(uint8_t cs) {
  uint8_t req[4] = {0}, rsp[4] = {0};
  
  // read summary status register
  req[0] = REG_SUMSTAT << 2;
  req[3] = crc_spi(req);
  write_read_spi(cs, req, rsp);
  write_read_spi(cs, req, rsp);
  delay(3);
  write_read_spi(cs, req, rsp);
  return (rsp[0] & 0x3) == 0x1; // RS (return status) == 01
}

// device initialization error -------------------------------------------------
void init_err() {
  for (int i = 0;; i++) {
    digitalWrite(LED_L, i % 2 ? LOW : HIGH); // blink LED 10 Hz and halt
    delay(100);
  }
}

// initialize global variables -------------------------------------------------
void init_vars() { 
  trace(3, "init_vars\r\n");
  
  noInterrupts();
  data_rate = 100;
  samp_intv = 1000000.0 / SAMP_RATE;
  time_sync = data_ready = 0;
  time_ms = samp_count = 0;
  pps_tick = 0;
  interrupts();
}

// write sensor data to HOST ---------------------------------------------------
//
//  Data format
//
//  $IMU,<t_sync>,<time>,<Rx>,<Ry>,<Rz>,<Ax>,<Ay>,<Az>,<temp>,<stat>*<CS>\r\n
//
//  <t_sync>   : External time sync status (0:async, 1: PPS, 2: PPS + NMEA)
//  <time>     : Time tag (s) (time of day in UTC for <t_sync> = 2)
//  <Rx>,<Ry>,<Rz> : Gyroscope X,Y,Z data (deg/H)
//  <Ax>,<Ay>,<Az> : Accelerometer X,Y,Z data (g)
//  <temp>     : Sensor temperature (C)
//  <stat>     : Sensor summary status (HEX) (0: OK, 1: error)
//               bit 6-4: Rx,Ry,Rz, bit 2-0: Ax,Ay,Az
//  <CS>       : Checksum same as NMEA
//
void write_data(uint8_t t_sync, uint32_t t_ms, const double *gyro,
                const double *accl, const double *temp, uint8_t stat) {
    double gyro_c[3], accl_c[3];
    char buff[128], *p = buff;
    
    // cross-axis compensation ([1] 2.6)
    for (int i = 0; i < 3; i++) {
      gyro_c[i] = Cx[i][0] * gyro[0] + Cx[i][1] * gyro[1] + Cx[i][2] * gyro[2];
      accl_c[i] = Bx[i][0] * accl[0] + Bx[i][1] * accl[1] + Bx[i][2] * accl[2];
    }
    p += sprintf(p, "$IMU,%d,%.3f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.1f,%02X",
           t_sync, t_ms * 0.001, gyro_c[0], gyro_c[1], gyro_c[2], accl_c[0],
           accl_c[1], accl_c[2], (temp[0] + temp[1]) * 0.5, stat);
    p += sprintf(p, "*%02X\r\n", cs_nmea(buff, p - buff));
    
    Serial.print(buff);
}

// interval timer ISR ----------------------------------------------------------
void timer_ISR() {
  uint32_t pps_time = micros() - pps_tick;
  uint32_t pps_intv = ROUND(samp_intv) * SAMP_RATE;
  uint32_t thres = ROUND(samp_intv) / 2;
  
  if (time_sync > 0 && pps_time > pps_intv + thres) {
    trace(3, "PPS signal lost: time=%.3f\r\n", time_ms * 0.001);
    time_sync = 0; // external time sync mode = async
  }
  if (time_sync == 0 || (pps_time > thres && pps_time < pps_intv - thres)) {
    // read sensor data
    read_sens();
  }
  if (pps_time >= pps_intv / LED_DUTY) {
    digitalWrite(LED_PPS, LOW);
  }
}

// PPS ISR ---------------------------------------------------------------------
void pps_ISR() {
  uint32_t tick = micros();
  uint32_t pps_time = tick - pps_tick;
  pps_tick = tick;
  
  // restart interval timer
  TimerTc3.restart();
  
  // read sensor data
  read_sens();
  
  if (time_sync == 0) {
    trace(3, "PPS signal detected: time=%.3f\r\n", time_ms * 0.001);
    time_sync = 1; // external time sync mode = PPS
  }
  time_ms = time_ms / 1000 * 1000;
  samp_count = 0;
  
  // adjust samping interval
  if (pps_time > 990000 && pps_time < 1010000) {
    samp_intv += ((double)pps_time / SAMP_RATE - samp_intv) * 0.1;
    TimerTc3.setPeriod(ROUND(samp_intv));
  }
  digitalWrite(LED_PPS, HIGH);
}

// read sensor data ------------------------------------------------------------
void read_sens() {
  static const uint8_t reg1[] = {REG_DUE_RZ2, REG_DUE_RY2};
  static const uint8_t reg2[] = {REG_UNO_RX2, REG_UNO_AX, REG_UNO_AY, REG_UNO_AZ};
  uint16_t data1[2], data2[4];
  
  time_ms = (time_ms + 1000 / SAMP_RATE) % 86400000;
  
  if (!samp_enable) return;
  
  // read sensor registers
  if (!read_reg(CS_DUE, reg1, 2, data1) || !read_reg(CS_UNO, reg2, 4, data2)) {
    return;
  }
  double gyro[3], accl[3];
  gyro[0] = (int16_t)data2[0] * SCALE_GYRO * (rang_sel[0] ? 0.5  : 1.0);
  gyro[1] = (int16_t)data1[1] * SCALE_GYRO * (rang_sel[0] ? 0.5  : 1.0);
  gyro[2] = (int16_t)data1[0] * SCALE_GYRO * (rang_sel[0] ? 0.5  : 1.0);
  accl[0] = (int16_t)data2[1] * SCALE_ACCL * (rang_sel[1] ? 0.25 : 1.0);
  accl[1] = (int16_t)data2[2] * SCALE_ACCL * (rang_sel[1] ? 0.25 : 1.0);
  accl[2] = (int16_t)data2[3] * SCALE_ACCL * (rang_sel[1] ? 0.25 : 1.0);
  samp_count++;
  
  for (int i = 0; i < 3; i++) {
    if (samp_count == 1) { // first sensor data
      data_gyro[i] = gyro[i];
      data_accl[i] = accl[i];
    }
    else { // average sensor data
      data_gyro[i] += (gyro[i] - data_gyro[i]) / samp_count;
      data_accl[i] += (accl[i] - data_accl[i]) / samp_count;
    }
  }
  if (time_ms % (1000 / data_rate) == 0) {
    data_ready = 1;
    
    if (samp_count != SAMP_RATE / data_rate) {
      trace(2, "samp_count error: time=%.3f samp_count=%d\r\n", time_ms * 0.001,
        samp_count);
    }
  }
  digitalWrite(LED_L, time_ms % LED_INTV < LED_INTV / LED_DUTY ? LOW : HIGH);
}

// read sensor status ----------------------------------------------------------
uint8_t read_stat(double *temp) {
  static const uint8_t reg1[] = {REG_TEMP, REG_SUMSTAT};
  static const uint8_t reg2[] = {REG_TEMP, REG_SUMSTAT};
  uint16_t data1[2], data2[2];
  
  // read status registers
  if (!read_reg(CS_DUE, reg1, 2, data1) || !read_reg(CS_UNO, reg2, 2, data2)) {
    return 0xFF; // error
  }
  temp[0] = 25.0 + (int16_t)data1[0] * SCALE_TEMP;
  temp[1] = 25.0 + (int16_t)data2[0] * SCALE_TEMP;
  
  // sensor summary status (0x00: all OK)
  return (uint8_t)(((~data2[1] >> 12) & 0x1) << 6 | // S_OK_Rx_F
                   ((~data1[1] >> 14) & 0x1) << 5 | // S_OK_Ry_F
                   ((~data1[1] >> 12) & 0x1) << 4 | // S_OK_Rz_F
                   ((~data2[1] >> 10) & 0x1) << 2 | // S_OK_Ax_F
                   ((~data2[1] >>  7) & 0x1) << 1 | // S_OK_Ay_F
                   ((~data2[1] >>  4) & 0x1) << 0); // S_OK_Az_F
}

// read registers of SCHA63T ---------------------------------------------------
bool read_reg(uint8_t cs, const uint8_t *reg, int n, uint16_t *data) {
  uint8_t req[4] = {0}, rsp[4];
  
  digitalWrite(TEST_PIN, HIGH);
  
  for (int i = 0; i <= n; i++) {
    if (i < n) {
      req[0] = (reg[i] & 0x1F) << 2; // reg read operation
      req[3] = crc_spi(req);
    }
    write_read_spi(cs, req, rsp);
    
    if (i > 0) {
      if (crc_spi(rsp) != rsp[3]) { // CRC error
        digitalWrite(TEST_PIN, LOW);
        trace(2, "read_reg CRC error: cs=%d reg=%02X\r\n", cs, reg[i]);
        return false;
      }
      data[i-1] = ((uint16_t)rsp[1] << 8) | rsp[2];
    }
  }
  digitalWrite(TEST_PIN, LOW);
  return true;
}

// write register of SCHA63T ---------------------------------------------------
bool write_reg(uint8_t cs, uint8_t reg, uint16_t data) {
  uint8_t req[4], rsp[4];
  
  req[0] = (uint8_t)0x80 | ((reg & 0x1F) << 2); // reg write operation
  req[1] = (uint8_t)BYTE_H(data);
  req[2] = (uint8_t)BYTE_L(data);
  req[3] = crc_spi(req);
  write_read_spi(cs, req, rsp);
  
  req[0] = (reg & 0x1F) << 2; // reg read operation
  req[1] = req[2] = 0;
  req[3] = crc_spi(req);
  write_read_spi(cs, req, rsp);
  
  if (crc_spi(rsp) != rsp[3]) {
    trace(2, "write_reg CRC error: cs=%d reg=%02X\r\n", cs, reg);
    return false;
  }
  return true;
}

// write and read 32 bit SPI frame ---------------------------------------------
void write_read_spi(uint8_t cs, const uint8_t *req, uint8_t *rsp) {
  
  digitalWrite(cs, LOW);
  rsp[0] = SPI.transfer(req[0]);
  rsp[1] = SPI.transfer(req[1]);
  rsp[2] = SPI.transfer(req[2]);
  rsp[3] = SPI.transfer(req[3]);
  digitalWrite(cs, HIGH);
  
  trace(4, "read_spi: cs=%d req=%02X%02X%02X%02X rsp=%02X%02X%02X%02X\r\n",
    cs, req[0], req[1], req[2], req[3], rsp[0], rsp[1], rsp[2], rsp[3]);
}

// SPI frame CRC of SCHA63T ([1] 5.1.6) ----------------------------------------
uint8_t crc_spi(const uint8_t *data) {
  uint8_t crc = 0xFF;
  
  for (int i = 0; i < 3; i++) {
    for (int j = 7; j >= 0; j--) {
      uint8_t bit = crc >> 7;
      crc <<= 1;
      if (bit ^ ((data[i] >> j) & 0x01)) crc ^= 0x1D;
    }
  }
  return (uint8_t)~crc;
}

// read command ----------------------------------------------------------------
void read_command() {
  int c;
  
  while ((c = Serial.read()) != -1) { // read HOST serial
    if (p_command >= sizeof(buff_command) - 1) {
      p_command = 0;
      continue;
    }
    buff_command[p_command++] = (char)c;
    if (c == '\r' || c == '\n') {
      buff_command[p_command] = '\0';
      parse_command(buff_command);
      p_command = 0;
    }
  }
}

// parse command ---------------------------------------------------------------
void parse_command(const char *buff) {
  static const char *command[] = {
    "reset", "device", "start", "stop", "status", "rate", "range", "filter",
    "read", "write", "debug", "table", "help", ""
  };
  static const command_func_t *command_func[] = {
    command_reset, command_device, command_start , command_stop , command_status,
    command_rate , command_range , command_filter, command_read , command_write ,
    command_debug, command_table , command_help
  };
  trace(3, "parse_command: buff=%s\r\n", buff);
  
  for (int i = 0; *command[i]; i++) {
    if (strncmp(buff, command[i], strlen(command[i]))) continue;
    command_func[i](buff);
    return;
  }
  trace(0, "> command error: %s\r\n", buff);
}

// command - reset -------------------------------------------------------------
void command_reset(const char *buff) {
  uint8_t ena = samp_enable;
  samp_enable = 0;
  init_device();
  init_vars();
  samp_enable = ena;
}

// command - device ------------------------------------------------------------
void command_device(const char *buff) {
  static const uint8_t reg[] = {REG_CID, REG_TR2, REG_TR0, REG_TR1};
  uint16_t data[4];
   
  if (!read_reg(CS_UNO, reg, 4, data)) {
    trace(0, "> device register read error\r\n");
  }
  else {
    trace(0, "> %-16s: %s\r\n", "device name", DEV_NAME);
    trace(0, "> %-16s: %s\r\n", "F/W version", FW_VER);
    trace(0, "> %-16s: %d\r\n", "component ID", data[0]);
    trace(0, "> %-16s: %05d%X%04XH00\r\n", "serial number", data[3],
      (data[1] >> 8) & 0xF, data[2]);
  }
}

// command - start -------------------------------------------------------------
void command_start(const char *buff) {
  samp_enable = 1;
}

// command - stop --------------------------------------------------------------
void command_stop(const char *buff) {
  samp_enable = 0;
}

// command - status ------------------------------------------------------------
void command_status(const char *buff) {
    static const char *str_tsync[] = {"async", "PPS", "PPS+NMEA"};
    double time = time_ms * 0.001, hms[3];
    
    hms[0] = floor(time / 3600.0);
    hms[1] = floor((time - hms[0] * 3600.0) / 60.0);
    hms[2] = time - hms[0] * 3600.0 - hms[1] * 60.0;
    trace(0, "> %-16s: %9.3f s (%02.0f:%02.0f:%06.3f)\r\n", "time", time, hms[0], hms[1], hms[2]);
    trace(0, "> %-16s: %s\r\n", "ext time sync", str_tsync[time_sync]);
    trace(0, "> %-16s: %9.4f,%9.4f,%9.4f deg/H\r\n", "gyro data", data_gyro[0], data_gyro[1], data_gyro[2]);
    trace(0, "> %-16s: %9.6f,%9.6f,%9.6f g\r\n", "accel data", data_accl[0], data_accl[1], data_accl[2]);
    trace(0, "> %-16s: %9.3f us\r\n", "sample interval", samp_intv);
}

// command - rate --------------------------------------------------------------
void command_rate(const char *buff) {
  int rate;
  
  if (sscanf(buff, "rate %d", &rate) < 1) {
    trace(0, "> %-16s: %d Hz\r\n", "sampling rate", SAMP_RATE);
    trace(0, "> %-16s: %d Hz\r\n", "output data rate", data_rate);
  }
  else if (rate <= 0 || SAMP_RATE % rate || rate > MAX_RATE) {
    trace(0, "> output data rate error: %d\r\n", rate);
  }
  else {
    noInterrupts();
    data_rate = rate;
    interrupts();
  }
}

// command - range -------------------------------------------------------------
void command_range(const char *buff) {
  static const char *str_rang1[] = {"300 deg/H", "150 deg/H"};
  static const char *str_rang2[] = {"6 g", "1.5 g"};
  int sel[2];
  
  if (sscanf(buff, "range %d %d", sel, sel + 1) < 2) {
    trace(0, "> %-16s: %d(%s)\r\n", "dyn-range gyro", rang_sel[0], str_rang1[rang_sel[0]]);
    trace(0, "> %-16s: %d(%s)\r\n", "dyn-range accel", rang_sel[1], str_rang2[rang_sel[1]]);
  }
  else if (sel[0] < 0 || sel[0] > 1 || sel[1] < 0 || sel[1] > 1) {
    trace(0, "> dynamic range error\r\n");
  }
  else {
    uint8_t ena = samp_enable;
    samp_enable = 0;
    rang_sel[0] = sel[0];
    rang_sel[1] = sel[1];
    init_device();
    samp_enable = ena;
  }
}

// command - filter ------------------------------------------------------------
void command_filter(const char *buff) {
  static const char *str_filt[] = {"13 Hz", "20 Hz", "46 Hz", "200 Hz", "300 Hz"};
  int sel[2];
  
  if (sscanf(buff, "filter %d %d", sel, sel + 1) < 2) {
    trace(0, "> %-16s: %d(%s)\r\n", "filter gyro", filt_sel[0], str_filt[filt_sel[0]]);
    trace(0, "> %-16s: %d(%s)\r\n", "filter accel", filt_sel[1], str_filt[filt_sel[1]]);
  }
  else if (sel[0] < 0 || sel[0] > 4 || sel[1] < 0 || sel[1] > 4) {
    trace(0, "> filter error: %d %d\r\n", sel[0], sel[1]);
  }
  else {
    uint8_t ena = samp_enable;
    samp_enable = 0;
    filt_sel[0] = sel[0];
    filt_sel[1] = sel[1];
    init_device();
    samp_enable = ena;
  }
}

// command - table -------------------------------------------------------------
void command_table(const char *buff) {
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "cross-axis gyro", Cx[0][0], Cx[0][1], Cx[0][2]);
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "", Cx[1][0], Cx[1][1], Cx[1][2]);
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "", Cx[2][0], Cx[2][1], Cx[2][2]);
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "cross-axis accel", Bx[0][0], Bx[0][1], Bx[0][2]);
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "", Bx[1][0], Bx[1][1], Bx[1][2]);
    trace(0, "> %-16s: %10.7f %10.7f %10.7f\r\n", "", Bx[2][0], Bx[2][1], Bx[2][2]);
}

// command - read --------------------------------------------------------------
void command_read(const char *buff) {
  char str[16] = "";
  uint8_t cs, reg[32];
  uint16_t data[32] = {0};
  int addr = -1, n = 0, k = 0;
  
  sscanf(buff, "read %15s %X", str, &addr);
  
  if      (!strcmp(str, "DUE")) cs = CS_DUE;
  else if (!strcmp(str, "UNO")) cs = CS_UNO;
  else {
    trace(0, "> device ASIC error: %s\r\n", str);
    return;
  }
  if (addr == -1) { // all registers
    for (int i = 0; i < 32; i++) reg[n++] = (uint8_t)i;
  }
  else if (addr < 0 || addr > 0x1F) {
    trace(0, "> %s register address error: %02X\r\n", str, addr);
    return;
  }
  else {
    reg[0] = (uint8_t)addr;
    n = 1;
  }
  if (!read_reg(cs, reg, n, data)) {
    trace(0, "> %s register(s) read error\r\n", str);
    return;
  }
  for (int i = 0; k < n; i++) {
    char buff[128], *p = buff;
    
    for (int j = 0; j < 8 && k < n; j++, k++) {
      p += sprintf(p, " %02X:%04X", reg[k], data[k]);
    }
    trace(0, "> %s %-12s:%s\r\n", str, "register", buff);
  }
}

// command - write -------------------------------------------------------------
void command_write(const char *buff) {
  char str[16] = "";
  int addr, val;
  uint8_t cs;
  
  if (sscanf(buff, "write %15s %X %X", str, &addr, &val) < 3) {
    trace(0, "> write command error\r\n");
    return;
  }
  if      (!strcmp(str, "DUE")) cs = CS_DUE;
  else if (!strcmp(str, "UNO")) cs = CS_UNO;
  else {
    trace(0, "> device ASIC error: %s\r\n", str);
    return;
  }
  if (addr < 0 || addr > 0x1F) {
    trace(0, "> device register address error: %s %02X\r\n", str, addr);
  }
  else if (!write_reg(cs, (uint8_t)addr, (uint16_t)val)) {
    trace(0, "> device register write error: %s %02X\r\n", str, addr);
  }
}

// command - debug -------------------------------------------------------------
void command_debug(const char *buff) {
  int level;
  
  if (sscanf(buff, "debug %d", &level) < 1) {
    trace(0, "> %-16s: %d\r\n", "debug level", debug_level);
  }
  else {
    noInterrupts();
    debug_level = (uint8_t)level;
    interrupts();
  }
}

// command - help --------------------------------------------------------------
void command_help(const char *buff) {
  trace(0, "> %-28s%s\r\n", "reset", ": Reset device.");
  trace(0, "> %-28s%s\r\n", "device", ": Show device information.");
  trace(0, "> %-28s%s\r\n", "start", ": Start output data.");
  trace(0, "> %-28s%s\r\n", "stop", ": Stop output data.");
  trace(0, "> %-28s%s\r\n", "status", ": Show status.");
  trace(0, "> %-28s%s\r\n", "rate [<rate>]", ": Set output data rate (Hz).");
  trace(0, "> %-28s%s\r\n", "range [<range_g> <range_a>]", ": Select dyn-range gyro/accel.");
  trace(0, "> %-28s%s\r\n", "filter [<filt_g> <filt_a>]", ": Select filter gyro/accel.");
  trace(0, "> %-28s%s\r\n", "table", ": Show cross-axis compensation table.");
  trace(0, "> %-28s%s\r\n", "read {DUE|UNO} [<addr>]", ": Read device register(s).");
  trace(0, "> %-28s%s\r\n", "write {DUE|UNO} <addr> <val>", ": Write device register.");
  trace(0, "> %-28s%s\r\n", "debug [<level>]", ": Set debug level.");
  trace(0, "> %-28s%s\r\n", "help", ": Show command help.");
}

// read NMEA -------------------------------------------------------------------
void read_nmea() {
  int c;
  
  while ((c = Serial1.read()) != -1) { // read NMEA serial (UART)
    if (p_nmea >= sizeof(buff_nmea) - 1) {
      p_nmea = 0;
      continue;
    }
    if (c == '$') {
      p_nmea = 0;
    }
    buff_nmea[p_nmea++] = (char)c;
    if (c == '\n') {
      buff_nmea[p_nmea] = '\0';
      if (time_sync > 0) parse_nmea(buff_nmea);
      p_nmea = 0;
    }
  }
}

// parse NMEA sentense ---------------------------------------------------------
void parse_nmea(const char *buff) {
  
  trace(3, "parse_nmea: buff=%s\r\n", buff);
  
  // test NMEA checksum
  int len = strlen(buff), sum;
  uint8_t cs = cs_nmea(buff, len - 5);
  if (len < 10 || sscanf(buff + len - 4, "%X", &sum) != 1 || cs != sum) {
    return;
  }
  // test NMEA GGA or RCM
  if (strncmp(buff + 3, "GGA", 3) && strncmp(buff + 3, "RMC", 3)) {
    return;
  }
  // parse NMEA
  const char *p = buff;
  int time[3] = {0};
  
  for (int i = 0; *p; i++, p++) {
    if (!(p = strchr(p, ','))) break;
    if (i == 0) sscanf(p, ",%2d%2d%2d", time, time + 1, time + 2);
  }
  noInterrupts();
  time_sync = 2; // external time sync mode = PPS+NMEA
  time_ms = (((time[0] * 60) + time[1]) * 60 + time[2]) * 1000 + time_ms % 1000;
  interrupts();
  
  trace(3, "parse_nmea: time=%02d:%02d:%02d\r\n", time[0], time[1], time[2]);
}

// NMEA checksum ---------------------------------------------------------------
uint8_t cs_nmea(const char *buff, size_t len) {
  uint8_t sum = 0;
  
  for (int i = 1; i < len; i++) {
    sum ^= buff[i];
  }
  return sum;
}

// printf to HOST serial -------------------------------------------------------
void trace(uint8_t level, const char *format, ...) {
  char buff[256];
  va_list arg;
  
  if (level > debug_level) return;
  va_start(arg, format);
  vsnprintf(buff, sizeof(buff), format, arg);
  va_end(arg);
  Serial.print(buff);
}

