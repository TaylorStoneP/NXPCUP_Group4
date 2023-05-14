#ifndef INC_STM32_UTILS_H_
#define INC_STM32_UTILS_H_

#include "main.h"

#define True 1
#define False 0

#define SOFTCLK_TIMER_TYPE htim1

extern TIM_HandleTypeDef SOFTCLK_TIMER_TYPE;

typedef struct{
	uint32_t time_ms;
	uint8_t flag;
	uint8_t auto_schedule;
	uint32_t period;
} Schedule;
extern Schedule CurrentTime;

typedef enum SCHEDULES{
	SCH_LDR_CHECK,
	SCH_CALIBRATION_CPLT,
	SCH_LED_TOGGLE,
	SCH_PULSE,
	SCH_SETSPEED,
	SCH_LEDTOP,
	SCH_LEDMID,
	SCH_LEDBOT,
	SCH_LEDOFF,
	SCHEDULE_N
}SCHEDULES;
extern Schedule ScheduleQueue[SCHEDULE_N];

#define SCHEDULE_HANDLE(task) 	if(ScheduleQueue[task].flag){ \
		ScheduleQueue[task].flag=0;

void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re, uint32_t offset);
void ScheduleTaskStop(SCHEDULES schedule);
void TaskScheduleSoftClock();
void TaskScheduleHandler();
void TaskScheduleSoftClock_FlagSet();

int32_t clamp(int32_t value, int32_t min, int32_t max);

#endif /* INC_STM32_UTILS_H_ */
