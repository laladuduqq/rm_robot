#ifndef __CHASSISCMD_H__
#define __CHASSISCMD_H__

#include <stdint.h>
void chassis_task_init(void);

typedef struct {
    uint8_t is_being_hit;      // 是否正在被击打
    uint16_t last_hp;          // 上一次的血量
    uint32_t last_hit_time;    // 上一次被击打的时间
    uint8_t is_first_check;    // 首次检查标志
} Hit_Check_t;

// 函数声明
uint8_t CheckRobotBeingHit(Hit_Check_t *hit_check);
#endif /* __CHASSISCMD_H__ */
