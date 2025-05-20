#ifndef __NVIC_H
#define __NVIC_H

#ifdef __cplusplus
extern "C" {
#endif

#define NVIC_PRIORITY_GROUPING NVIC_PRIORITYGROUP_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING))))>>4)

#ifdef __cplusplus
}
#endif

#endif /* __NVIC_H */