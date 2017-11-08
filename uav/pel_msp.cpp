#include <Arduino.h>

#include "pel_log.h"
#include "pel_msp.h"

size_t
pel_msp_send(uint8_t opcode, const uint8_t *data, size_t n)
{
    int bytes = 0;
    uint8_t checksum = 0;

    bytes = Naze32Serial.write((byte *) "$M<", 3);
 
    bytes += Naze32Serial.write(n);
    checksum ^= n;

    bytes += Naze32Serial.write(opcode);
    checksum ^= opcode;
    

    bytes += Naze32Serial.write(checksum);

    
    return bytes;
}

size_t
pel_msp_recv(uint8_t *buf, size_t n)
{
    int8_t b;
    size_t data_len;
    uint8_t command_code;
    uint8_t checksum;
    size_t offset;
    enum {
        sw_idle,
        sw_header_start,
        sw_header_m,
        sw_header_arrow,
        sw_header_size,
        sw_header_cmd
    } state;

    delay(60);

    state = sw_idle;
   
    while (Naze32Serial.available() > 0) {
        b = Naze32Serial.read();
        if (b == -1)
            continue;

        switch (state) {
            case sw_idle:
                switch (b) {
                    case '$':
                        state = sw_header_start;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

            case sw_header_start:
                switch (b) {
                    case 'M':
                        state = sw_header_m;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

            case sw_header_m:
                switch (b) {
                    case '>':
                        state = sw_header_arrow;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

                /* if sw_header_start begin message size check */
            case sw_header_arrow:
                data_len = b;

                /*
                 * Now we are expecting the payload size.  If the size indicated is
                 * larger than the buffer, dump the payload.
                 */
                if (data_len > n) {
                    state = sw_idle;
                    break;
                } else {
                    offset = 0;

                    checksum = 0;
                    checksum ^= (uint8_t) b;

                    state = sw_header_size;
                    break;
                }

                break;

            case sw_header_size:
                command_code = (uint8_t) b;
                checksum ^= (uint8_t) b;

                state = sw_header_cmd;
                break;

            case sw_header_cmd:
                if (offset < data_len) {
                    checksum ^= (uint8_t) b;
                    buf[offset++] = (uint8_t) b;
                    /* else command loops here until offset is no longer less that data_len */
                } else {
                    /*
                     * Verify that checksum and b are of the same value.  Then,
                     * assuming true, compare command_code against available
                     * commands in pel_msp.h and execute related command
                     */
                    if (checksum == b) {
                        /* TODO */
                    }

                    state = sw_idle;
                }

                break;
        }
    }

    return offset;
}

void
pel_msp_attitude(uint8_t *buf,Attitude* attiude)
{
    int16_t angx = readint16(buf);
    int16_t angy = readint16(buf + sizeof (int16_t));
    int16_t heading = readint16(buf + 2 * sizeof (int16_t));

    attiude->angx = angx;
    attiude->angy = angy;
    attiude->heading = heading;

    pel_log_debug("Attitude Recieved: ");
    pel_log_debug(angx);
    pel_log_debug(", ");
    pel_log_debug(angy);
    pel_log_debug(", ");
    pel_log_debug(heading);
    pel_log_debug("\n");
}

void
pel_msp_raw_imu(uint8_t *buf,IMUValues* imuValues)
{
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;
    int16_t magx;
    int16_t magy;
    int16_t magz;

    accx = readint16(buf);
    accy = readint16(buf + 2);
    accz = readint16(buf + 4);
    gyrx = readint16(buf + 6);
    gyry = readint16(buf + 8);
    gyrz = readint16(buf + 10);
    magx = readint16(buf + 12);
    magy = readint16(buf + 14);
    magz = readint16(buf + 16);

    imuValues->accx = accx;
    imuValues->accy = accy;
    imuValues->accz = accz;
    imuValues->gyrx = gyrx;
    imuValues->gyry = gyry;
    imuValues->gyrz = gyrz;
    imuValues->magx = magx;
    imuValues->magy = magy;
    imuValues->magz = magz;

    pel_log_debug("Raw GPS Recieved: ");
    pel_log_debug(accx);
    pel_log_debug(", ");
    pel_log_debug(accy);
    pel_log_debug(", ");
    pel_log_debug(accz);
    pel_log_debug(", ");
    pel_log_debug(gyrx);
    pel_log_debug(", ");
    pel_log_debug(gyry);
    pel_log_debug(", ");
    pel_log_debug(gyrz);
    pel_log_debug(", ");
    pel_log_debug(magx);
    pel_log_debug(", ");
    pel_log_debug(magy);
    pel_log_debug(", ");
    pel_log_debug(magz);
    pel_log_debug("\n");
}

void
pel_msp_raw_gps(uint8_t *buf)
{
    uint8_t fix;
    uint8_t num_sat;
    uint32_t lat;
    uint32_t lng;
    uint16_t altitude;
    uint16_t pel_speed;
    uint16_t ground_course;

    fix = buf[0];
    num_sat = buf[1];
    lat = readuint32(buf + 2);
    lng = readuint32(buf + 6);
    altitude = readuint16(buf + 10);
    pel_speed = readuint16(buf + 12);
    ground_course = readuint16(buf + 14);

    pel_log_debug("Raw GPS Recieved: ");
    pel_log_debug(fix);
    pel_log_debug(", ");
    pel_log_debug(num_sat);
    pel_log_debug(", ");
    pel_log_debug(lat);
    pel_log_debug(", ");
    pel_log_debug(lng);
    pel_log_debug(", ");
    pel_log_debug(altitude);
    pel_log_debug(", ");
    pel_log_debug(pel_speed);
    pel_log_debug(", ");
    pel_log_debug(ground_course);
    pel_log_debug("\n");
}

void
pel_msp_altitude(uint8_t *buf, Altitude* altitude)
{
    int32_t est_alt;
    int16_t vario;

    est_alt = readint16(buf);
    vario = readint16(buf + sizeof (int32_t));

    altitude->est_alt = est_alt;
    altitude->vario = vario;

    pel_log_debug("Altitude Recieved: ");
    pel_log_debug(est_alt);
    pel_log_debug(", ");
    pel_log_debug(vario);
    pel_log_debug("\n");
}

void
pel_msp_rc(uint8_t *buf,RCValues* rcValues)
{
    int c;
    uint16_t a[16];

    for (c = 0; c < 16; c++)
        a[c] = readuint16(buf + 2 * c);

    rcValues->rc_values = a;

    pel_log_debug("RC Values: ");

    for (c = 0; c < 16; c++) {
        pel_log_debug(a[c]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_ident(uint8_t *buf)
{
    uint8_t ver;
    uint8_t multitype;
    uint8_t msp_version;
    uint32_t capability;

    ver = buf[0];
    multitype = buf[1];
    msp_version = buf[2];
    capability = readuint32(buf + 3);

    pel_log_debug(ver);
    pel_log_debug(multitype);
    pel_log_debug(msp_version);
    pel_log_debug(capability);
    pel_log_debug("\n");
}

void
pel_msp_status(uint8_t *buf)
{
    uint16_t cycle_time;
    uint16_t i2c_errors_count;
    uint16_t sensor;
    uint32_t flag;
    uint8_t global_conf_currentset;

    cycle_time = readuint16(buf);
    i2c_errors_count = readuint16(buf + 2);
    sensor = readuint16(buf + 4);
    flag = readuint32(buf + 6);
    global_conf_currentset = buf[10];

    pel_log_debug("Status Recieved: ");
    pel_log_debug(cycle_time);
    pel_log_debug(", ");
    pel_log_debug(i2c_errors_count);
    pel_log_debug(", ");
    pel_log_debug(sensor);
    pel_log_debug(", ");
    pel_log_debug(flag);
    pel_log_debug(", ");
    pel_log_debug(global_conf_currentset);
    pel_log_debug("\n");
}

void
pel_msp_servo(uint8_t *buf)
{
    int i;
    uint16_t a[16];

    for (i = 0; i < 16; i++)
        a[i] = readuint16(buf + 2 * i);

    pel_log_debug("Servos: ");

    for (i = 0; i < 16; i++) {
        pel_log_debug(a[i]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_motor(uint8_t *buf)
{
    int i;
    uint16_t a[16];

    for (i = 0; i < 16; i++)
        a[i] = readuint16(buf + 2 * i);

    pel_log_debug("Motors: ");

    for (i = 0; i < 16; i++) {
        pel_log_debug(a[i]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_comp_gps(uint8_t *buf)
{
    uint16_t distance_to_home;
    uint16_t direction_to_home;
    uint8_t update;

    distance_to_home = readuint16(buf);
    direction_to_home = readuint16(buf + 2);
    update = buf[4];

    pel_log_debug("Home GPS Recieved: ");
    pel_log_debug(distance_to_home);
    pel_log_debug(", ");
    pel_log_debug(direction_to_home);
    pel_log_debug(", ");
    pel_log_debug(update);
    pel_log_debug("\n");
}

void
pel_msp_analog(uint8_t *buf,Analog* analog)
{
    uint8_t vbat;
    uint16_t power_meter_sum;
    uint16_t rssi;
    uint16_t amperage;

    vbat = buf[0];
    power_meter_sum = readuint16(buf + 1);
    rssi = readuint16(buf + 3);
    amperage = readuint16((buf + 5));

    analog->vbat = vbat;
    analog->power_meter_sum = power_meter_sum;
    analog->rssi = rssi;
    analog->amperage = amperage;

    /*pel_log_debug(vbat);
    pel_log_debug(",");
    pel_log_debug(power_meter_sum);
    pel_log_debug(",");
    pel_log_debug(rssi);
    pel_log_debug(",");
    pel_log_debug(amperage);
    pel_log_debug("\n");*/
}

void
pel_msp_rc_tuning(uint8_t *buf)
{
    uint8_t rc_rate;
    uint8_t rc_expo;
    uint8_t roll_pitch_rate;
    uint8_t yaw_rate;
    uint8_t dyn_thr_pid;
    uint8_t throttle_mid;
    uint8_t throttle_expo;

    rc_rate = buf[0];
    rc_expo = buf[1];
    roll_pitch_rate = buf[2];
    yaw_rate = buf[3];
    dyn_thr_pid = buf[4];
    throttle_mid = buf [5];;
    throttle_expo = buf[6];

    pel_log_debug("RC Tuning: ");
    pel_log_debug(rc_rate);
    pel_log_debug(",");
    pel_log_debug(rc_expo);
    pel_log_debug(",");
    pel_log_debug(roll_pitch_rate);
    pel_log_debug(",");
    pel_log_debug(yaw_rate);
    pel_log_debug(",");
    pel_log_debug(dyn_thr_pid);
    pel_log_debug(",");
    pel_log_debug(throttle_mid);
    pel_log_debug(",");
    pel_log_debug(throttle_expo);
    pel_log_debug("\n");
}


#if 0
//not certain of the next two functions
void
pel_msp_pid(uint8_t *buf)
{
    int i;
    uint8_t a[30];

    for (i = 0; i < 30; i++)
        a[i] = buf[i];

    pel_log_debug("PID: ");

    for (i = 0; i < 30; i++) {
        pel_log_debug(a[i]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_box(uint8_t *buf, int box)
{
    int i;
    uint16_t a;
    for (i = 0; i < box; i++)
        a[i] = readuint16(buf + 2 * i);

    pel_log_debug("Box Items: ");

    for (i = 0; i < box; i++) {
        pel_log_debug(a[i]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}
#endif

void
pel_msp_misc(uint8_t *buf)
{
    uint16_t power_trigger;
    uint16_t min_throttle;
    uint16_t max_throttle;
    uint16_t min_command;
    uint16_t failsafe_throttle;
    uint16_t arm;
    uint32_t lifetime;
    uint16_t mag_declination;
    uint8_t vbat_scale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;

    power_trigger = readuint16(buf);
    min_throttle = readuint16(buf + 2);
    max_throttle = readuint16(buf + 4);
    min_command = readuint16(buf + 6);
    failsafe_throttle = readuint16(buf + 8);
    arm = readuint16(buf + 10);
    lifetime = readuint32(buf + 12);
    mag_declination = readuint16(buf + 16);
    vbat_scale = buf[18];
    vbatlevel_warn1 = buf[19];
    vbatlevel_warn2 = buf[20];
    vbatlevel_crit = buf[21];

    pel_log_debug("Miscellaneous: ");
    pel_log_debug(power_trigger);
    pel_log_debug(",");
    pel_log_debug(min_throttle);
    pel_log_debug(",");
    pel_log_debug(min_command);
    pel_log_debug(",");
    pel_log_debug(failsafe_throttle);
    pel_log_debug(",");
    pel_log_debug(arm);
    pel_log_debug(",");
    pel_log_debug(lifetime);
    pel_log_debug(",");
    pel_log_debug(mag_declination);
    pel_log_debug(",");
    pel_log_debug(vbat_scale);
    pel_log_debug(",");
    pel_log_debug(vbatlevel_warn1);
    pel_log_debug(",");
    pel_log_debug(vbatlevel_warn2);
    pel_log_debug(",");
    pel_log_debug(vbatlevel_crit);
    pel_log_debug("\n");
}

void
pel_msp_motor_pins(uint8_t *buf)
{
    int i;
    uint8_t a[8];

    for (i = 0; i < 8; i++)
        a[i] = buf[i];

    pel_log_debug("Motor Pins: ");

    for (i = 0; i < 8; i++) {
        pel_log_debug(a[i]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_wp(uint8_t *buf)
{
    uint8_t wp_no;
    uint32_t lat;
    uint32_t lon;
    uint32_t alt_hold;
    uint16_t heading;
    uint16_t time_to_stay;
    uint8_t nav_flag;

    wp_no = buf[0];
    lat = readuint32(buf + 1);
    lon = readuint32(buf + 5);
    alt_hold = readuint32(buf + 9);
    heading = readuint16(buf + 13);
    time_to_stay = readuint16(buf + 15);
    nav_flag = buf[17];

    pel_log_debug(wp_no);
    pel_log_debug(",");
    pel_log_debug(lat);
    pel_log_debug(",");
    pel_log_debug(lon);
    pel_log_debug(",");
    pel_log_debug(alt_hold);
    pel_log_debug(",");
    pel_log_debug(heading);
    pel_log_debug(",");
    pel_log_debug(time_to_stay);
    pel_log_debug(",");
    pel_log_debug(nav_flag);
    pel_log_debug("\n");
}

void
pel_msp_select_setting(uint8_t *buf)
{
    uint8_t current_set;

    current_set = buf[0];

    pel_log_debug("Current Selection: ");
    pel_log_debug(current_set);
    pel_log_debug("\n");
}

