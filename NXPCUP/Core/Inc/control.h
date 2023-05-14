/*
 * control.h
 *
 *  Created on: Apr 3, 2023
 *      Author: TheDa
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "mathops.h"

#define N_LDR_NODES 8

typedef struct LDR_NODE{
	RUNTIME_AVERAGE hrave;
}LDR_NODE;

typedef struct MOTOR_CONTROLS{
	int32_t steer;
	int32_t speed;
}MOTOR_CONTROLS;

enum LDR{
	LDR_L1 = 0,
	LDR_L2 = 1,
	LDR_L3 = 2,
	LDR_R1 = 3,
	LDR_R2 = 4,
	LDR_R3 = 5,
	LDR_F = 6,
};

extern LDR_NODE LDR_nodes[N_LDR_NODES];
extern GPIO_TypeDef* LDR_LED_Ports[N_LDR_NODES];
extern uint16_t LDR_LED_Pins[N_LDR_NODES];
extern MOTOR_CONTROLS MotorControls;

void SetSteer(uint16_t value);
void SetSpeed(uint16_t value);

#endif /* INC_CONTROL_H_ */
