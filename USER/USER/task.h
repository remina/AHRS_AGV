#ifndef _TASK_H
#define _TASK_H

extern u8 sch_add_task(void(*pFunction)(), u16 delay, u16 period);
extern u8 sch_delete_task(u8 index);
extern void sch_dispatch_tasks(void);
extern void sch_init(void);
extern void sch_start(void);

#endif

