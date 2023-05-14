/*
 * mathops.c
 *
 *  Created on: Apr 17, 2023
 *      Author: TheDa
 */


#include "mathops.h"
#include "stm32_utils.h"

void Integrate(DiscreteIntegratorHandle* integrator, int32_t new_value){
	integrator->value = integrator->value + new_value;
	integrator->count++;
}

void Derivative(DiscreteDerivativeHandle* derivitive, int32_t new_value){
	derivitive->value = ((new_value-derivitive->kminus1)*8 + (derivitive->kminus1-derivitive->kminus2)*5 + (derivitive->kminus2-derivitive->kminus3)*2)/15;
	derivitive->kminus3 = derivitive->kminus2;
	derivitive->kminus2 = derivitive->kminus1;
	derivitive->kminus1 = new_value;
	derivitive->count++;
}

void PID_SetGain(PID_Handle* hpid, int32_t p, int32_t i, int32_t d){
	hpid->gain_p = p;
	hpid->gain_i = i;
	hpid->gain_d = d;
}

void PID_Reset(PID_Handle* hpid){
	hpid->integral.value = 0;
	hpid->derivitive.value = 0;
}
void PID_UpdateSetpoint(PID_Handle* hpid, int32_t set_point){
	hpid->set_point = set_point;
}
int32_t PID_Compute(PID_Handle* hpid, int32_t current_point){
	int32_t error = hpid->set_point - current_point;
	Integrate(&hpid->integral, error);
	Derivative(&hpid->derivitive, error);
	return (hpid->gain_p * error + hpid->gain_i * hpid->integral.value + hpid->gain_d * hpid->derivitive.value)/1000;
}

int32_t PID_Compute_ForceDampen(PID_Handle* hpid, int32_t current_point){
	int32_t error = hpid->set_point - current_point;
	Integrate(&hpid->integral, error);
	Derivative(&hpid->derivitive, error);
	if(hpid->derivitive.value<0){
		hpid->derivitive.value*=-1;
	}
	if(error<0){
		error  *=-1;
	}
	return (hpid->gain_p * error + hpid->gain_i * hpid->integral.value + hpid->gain_d * hpid->derivitive.value)/1000;
}

int32_t PID_Compute_Distance(PID_Handle* hpid, int32_t current_point){
	int32_t error = hpid->set_point - current_point;
	Integrate(&hpid->integral, error);
	Derivative(&hpid->derivitive, error);
	if(error<0){
		error = 0;
	}else{
		hpid->integral.value = 0;
	}
	return (hpid->gain_p * error + hpid->gain_i * hpid->integral.value + hpid->gain_d * hpid->derivitive.value)/1000;
}

void RUNTIME_AVERAGE_Update(RUNTIME_AVERAGE* hrave, uint32_t value){
	hrave->n_samples++;
	hrave->average =  ((hrave->average*10000) - (hrave->average*10000)/hrave->n_samples + (value*10000)/hrave->n_samples)/10000;
}
void RUNTIME_AVERAGE_Reset(RUNTIME_AVERAGE* hrave){
	hrave->n_samples=1;
}
