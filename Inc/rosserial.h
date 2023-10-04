/*
 * rosserial.h
 *
 *  Created on: May 8, 2023
 *      Author: yeonwoo
 */

#ifndef INC_ROSSERIAL_H_
#define INC_ROSSERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

void setup();
void loop();

extern volatile int32_t tar_speed_R;
extern volatile int32_t tar_speed_L;

#ifdef __cplusplus
}
#endif

#endif /* INC_ROSSERIAL_H_ */
