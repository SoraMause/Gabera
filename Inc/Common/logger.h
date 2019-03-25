#ifndef __LOGGER_H
#define __LOGGER_H

#include <stdint.h>

#define LOG_NUMBER 4096

typedef struct {
  int16_t trans_ideal_vel[LOG_NUMBER];
  int16_t trans_vel[LOG_NUMBER];
  //int16_t rotation_ideal[LOG_NUMBER];
  //int16_t rotation_vel[LOG_NUMBER];
  float trans_dis[LOG_NUMBER];
  int16_t sensor_left[LOG_NUMBER];
  int16_t sensor_right[LOG_NUMBER];
  //int16_t sensor_front[LOG_NUMBER];
  //int8_t batt_data[LOG_NUMBER];
}t_log_data;

extern t_log_data logger;

void log_init( void );
void setLog( void );
void showLog( void );
void setLogFlag( int8_t _flag );

#endif /* __LOGGER_H */