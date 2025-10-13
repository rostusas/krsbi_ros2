/*
 * utility.c
 *
 *  Created on: Jul 20, 2025
 *      Author: musthofa
 */

#include "utility.h"
#include <math.h>

float error_sudut_calculation(float setPointSudut, float dataSudut){
	if(setPointSudut - dataSudut > 180) dataSudut += 360;
	else if ((dataSudut - setPointSudut > 180)) dataSudut -= 360;
	return setPointSudut - dataSudut;
}

float error_jarak_calculation(float setPointJarak, float dataJarak){
	return (dataJarak - setPointJarak);
}

void map(float *data, float inMin, float inMax, float outMin, float outMax){
	*data =  (*data - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

