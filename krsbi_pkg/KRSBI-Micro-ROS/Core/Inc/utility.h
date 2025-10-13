/*
 * utility.h
 *
 *  Created on: Jul 20, 2025
 *      Author: musthofa
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

float error_sudut_calculation(float setPointSudut, float dataSudut);
float error_jarak_calculation(float setPointJarak, float dataJarak);
void map(float *data, float inMin, float inMax, float outMin, float outMax);

#endif /* INC_UTILITY_H_ */
