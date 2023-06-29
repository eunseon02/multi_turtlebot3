
#ifndef MAF
#define MAF

#include"MAF.hpp"

#pragma once

// #define average 1
// #define median 2


class Moving_Average {
public:
	Moving_Average();
	float LowPassFilter(float x, int y);
	int len = 10;
	float buff[10];
	
private:
	float sum = 0;
	float avrg = 0;
	float medn = 0;
	float temp = 0;

};
#endif