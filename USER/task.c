#include "stm32f10x.h"
#include "stdio.h"
#include "task.h"
#include "Time.h"

/* 任务调度数据结构 */
#define SCH_MAX_TASKS  16
typedef struct data{
	void (*pTask)(void);
	u16 delay;
	u16 period;
	u8 run;
}sTask;

sTask SCH_tasks[SCH_MAX_TASKS];

void sch_init(void)
{
	u8 i;
	//设置定时器，产生周期中断
	SysTick_Init();

	for (i = 0; i < SCH_MAX_TASKS; i++)
		sch_delete_task(i);			
}

void sch_start(void)
{
	SysTick_Start();
}

u8 sch_add_task(void(*pFunction)(), u16 delay, u16 period)
{
	u8 index = 0;
	while ((SCH_tasks[index].pTask != 0) && (index < SCH_MAX_TASKS))
		index++;
	if (index == SCH_MAX_TASKS)
	{
		//返回错误码
		return 8;
	}

	SCH_tasks[index].pTask = pFunction;
	SCH_tasks[index].delay = delay;
	SCH_tasks[index].period = period;
	SCH_tasks[index].run = 0;
	return index;
}

u8 sch_delete_task(u8 index)
{
	if (SCH_tasks[index].pTask == 0)
		//返回错误码
		return 8;

	SCH_tasks[index].pTask = NULL;
	SCH_tasks[index].delay = 0;
	SCH_tasks[index].period = 0;
	SCH_tasks[index].run = 0;
	return 0;
}

void sch_go_to_sleep(void)
{
	return;
}

void sch_dispatch_tasks(void)
{
	u8 index;
	for (index = 0; index < SCH_MAX_TASKS; index++)
	{
		if (SCH_tasks[index].run > 0)
		{
			(*SCH_tasks[index].pTask)();
			SCH_tasks[index].run = 0;
			if (SCH_tasks[index].period == 0)
				sch_delete_task(index);
		}
	}

	//报告系统状况
//	sch_report_status();

	//调度器进入空闲模式
	sch_go_to_sleep();
}


void sch_update(void)
{
	u8 index;

	for (index = 0; index < SCH_MAX_TASKS; index++)
	{
		if (SCH_tasks[index].pTask)
		{
			if (SCH_tasks[index].delay == 0)
			{
				SCH_tasks[index].run = 1;
				if (SCH_tasks[index].period != 0)
					SCH_tasks[index].delay = SCH_tasks[index].period;
			}
			else
				SCH_tasks[index].delay--;
		}
	}
}





