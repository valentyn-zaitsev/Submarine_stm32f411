#ifndef INC_ADD_H_
#define INC_ADD_H_

#include <stdint.h>

typedef struct {
  uint16_t leftX;
  uint16_t leftY;
  uint16_t rightX;
  uint16_t rightY;
} joystick_t;

typedef enum {
	PUMP_OFF,
	PUMP_PUSH,
	PUMP_PULL
}pump_states_t;

typedef enum {
	CAMERA_OFF,
	CAMERA_ON
}camera_states_t;






uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
int16_t constrain(int16_t value, int16_t min, int16_t max);

#endif /* INC_ADD_H_ */
