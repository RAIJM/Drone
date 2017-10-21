#ifndef PEL_PROTOCOL_H
#define PEL_PROTOCOL_H

#include <Arduino.h>

#include <stdint.h>

#define Naze32Serial    Serial3 /* serial0 */

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

/*
 * The argument m points to a buffer.  The buffer's contents will not be modified.
 * The argument is surrounded in parentheses because of pointer arithmetic.
 * These also do not perform array bounds checking.
 * 
 */
#define readint16(m)  (int16_t) ((m)[1] << 8 | (m)[0])
#define readuint16(m)           ((m)[1] << 8 | (m)[0])
#define readuint32(m)           ((m)[3] << 24 | (m)[2] << 16 | (m)[1] << 8 | (m)[0])
#define readint32(m)  (int32_t) ((m)[3] << 24 | (m)[2] << 16 | (m)[1] << 8 | (m)[0])

extern size_t pel_msp_send(uint8_t opcode, const uint8_t *data, size_t n);
extern size_t pel_msp_recv(uint8_t *buf, size_t n);

struct Attitude{
  
  int16_t angx;
    int16_t angy;
    int16_t heading;

    int16_t getAngX()
    {
      return angx;
    }

    int16_t getAngY()
    {
      return angy;
    }

    int16_t getHeading()
    {
      return heading;
    }

};


struct IMUValues{

  int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;
    int16_t magx;
    int16_t magy;
    int16_t magz;
};


struct Altitude{
  int32_t est_alt;
    int16_t vario;
};

extern void pel_msp_attitude(uint8_t *buf,Attitude* attitude);
extern void pel_msp_raw_gps(uint8_t *buf);
extern void pel_msp_raw_imu(uint8_t *buf, IMUValues* imuValues);
extern void pel_msp_altitude(uint8_t *buf, Altitude* altitude);
extern void pel_msp_rc(uint8_t *buf);
extern void pel_msp_ident(uint8_t *buf);
extern void pel_msp_status(uint8_t *buf);
extern void pel_msp_servo(uint8_t *buf);
extern void pel_msp_motor(uint8_t *buf);
extern void pel__msp_comp_gpx(uint8_t *buf);
extern void pel_msp_analog(uint8_t *buf);
extern void pel_msp_rc_tuning(uint8_t *buf);
extern void pel_msp_pid(uint8_t *buf);
extern void pel_msp_box(uint8_t *buf);
extern void pel_msp_misc(uint8_t *buf);
extern void pel_msp_motor_pins(uint8_t *buf);
extern void pel_msp_wp(uint8_t *buf);
extern void pel_msp_select_settings(uint8_t *buf);




#endif /* PEL_PROTOCOL_H */

