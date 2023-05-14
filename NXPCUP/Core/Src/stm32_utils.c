#include "stm32_utils.h"

Schedule CurrentTime;
Schedule ScheduleQueue[SCHEDULE_N];

uint8_t TaskSchedulerSoftClock_Flag;

void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re, uint32_t offset){
	if(period==0){
		ScheduleQueue[schedule].time_ms = 0xFFFFFFFF;
		ScheduleQueue[schedule].flag = True;
	}else if(period==0xFFFFFFFF){
		ScheduleQueue[schedule].time_ms = 0xFFFFFFFF;
	}
	else{
		ScheduleQueue[schedule].time_ms = (CurrentTime.time_ms+period+offset)%0xFFFFFFFF;
	}
    ScheduleQueue[schedule].auto_schedule = auto_re;
    ScheduleQueue[schedule].period = period;

}

void ScheduleTaskStop(SCHEDULES schedule){
	ScheduleTask(schedule, 0xFFFFFFFF, False, 0);
}

void TaskScheduleSoftClock(){
	if(!TaskSchedulerSoftClock_Flag){
		return;
	}
	TaskSchedulerSoftClock_Flag = 0;

    if(++CurrentTime.time_ms==0xFFFFFFFF){
    	CurrentTime.time_ms=0;
    }

    for(int task = 0; task<SCHEDULE_N; task++){
        if((ScheduleQueue[task].time_ms == CurrentTime.time_ms)){
            ScheduleQueue[task].flag = True;
            if(ScheduleQueue[task].auto_schedule){
                ScheduleTask(task, ScheduleQueue[task].period, True,0);
            }else{
                ScheduleQueue[task].time_ms = 0xFFFFFFFF;
            }
        }
    }
}

void TaskScheduleSoftClock_FlagSet(){
	TaskSchedulerSoftClock_Flag = 1;
}

int32_t clamp(int32_t value, int32_t min, int32_t max){
	if(value>max){
		return max;
	}
	if(value<min){
		return min;
	}
	return value;
}
