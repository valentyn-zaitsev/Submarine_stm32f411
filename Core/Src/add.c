
#include "add.h"

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int16_t constrain(int16_t value, int16_t min, int16_t max){
	if (value > max){
		return max;
	} else if (value < min){
		return min;
	} else {
		return value;
	}
}
