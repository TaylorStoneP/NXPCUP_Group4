/*
 * control.c
 *
 *  Created on: Apr 3, 2023
 *      Author: TheDa
 */

#include "control.h"
#include "stm32_utils.h"

LDR_NODE LDR_nodes[N_LDR_NODES];
GPIO_TypeDef* LDR_LED_Ports[N_LDR_NODES] = {LED_LDR_L1_GPIO_Port,LED_LDR_L2_GPIO_Port,LED_LDR_L3_GPIO_Port,LED_LDR_R1_GPIO_Port,LED_LDR_R2_GPIO_Port,LED_LDR_R3_GPIO_Port, LED_BRD_GPIO_Port, LED_BRD_GPIO_Port};
uint16_t LDR_LED_Pins[N_LDR_NODES] = {LED_LDR_L1_Pin,LED_LDR_L2_Pin,LED_LDR_L3_Pin,LED_LDR_R1_Pin,LED_LDR_R2_Pin,LED_LDR_R3_Pin, LED_BRD_Pin, LED_BRD_Pin};
MOTOR_CONTROLS MotorControls;

void SetSteer(uint16_t value){
	TIM2->CCR1 = 500 + value*2;
}
void SetSpeed(uint16_t value){
	TIM2->CCR2 = 1000 + value;
}
