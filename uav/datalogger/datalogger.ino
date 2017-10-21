
#include <stddef.h>

#include "pel_log.h"
#include "pel_msp.h"

#define INBUF_SIZE  64

static void setup(void);
static void loop(void);
static void getIMU(void);
static void getAttitude(void);
static void
setup(void)
{
#ifndef NDEBUG
    Serial.begin(115200);
#endif
    
    Naze32Serial.begin(115200);

    pel_log_debug("Setup");
    pel_log_debug("\n");
}

static void
loop(void)
{
    
    getIMU();
    //getAttitude();  
    
}

static void getIMU(void)
{
  uint8_t buf[INBUF_SIZE];
  uint8_t data = 0;
  
  IMUValues imuValues;
  (void) pel_msp_send(MSP_RAW_IMU, (uint8_t *) &data, 0);
  (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
  (void) pel_msp_raw_imu(buf,&imuValues);

  

  Serial.print("AccelX ");
  Serial.print(imuValues.accx);

  Serial.print("AccelY ");
  Serial.print(imuValues.accy);

  Serial.print("AccelZ ");
  Serial.println(imuValues.accz);

   // Serial.print("GRyY ");
   // Serial.println(imuValues.gyry);
}

static void getAttitude(void)
{
  uint8_t buf[INBUF_SIZE];
  uint8_t data = 0;
  
  Attitude attitude;
  (void) pel_msp_send(MSP_ATTITUDE, (uint8_t *) &data, 0);
  (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
  (void) pel_msp_attitude(buf,&attitude);

//  Serial.print("AngleX ");
//    Serial.println(attitude.angx);
//
//    Serial.print("AngleY ");
//    Serial.println(attitude.angy);

    Serial.print("Heading ");
    Serial.println(attitude.heading);
}


