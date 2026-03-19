#ifndef __BSP_RTOSDEBUG_H
#define __BSP_RTOSDEBUG_H
#include <stdint.h>

/*

使用说明：
1.使用该功能前，请提供变量 RtosDebugPrivateVar task1debugVar; //变量名称随意
2.使用接口： pRtosDebugInterface_t task1debug = &RTOSTaskDebug;
3.在某个任务计算任务频率： task1freq = task1debug->UpdateFreq(&task1debugVar); //获取的返回值即该任务的频率

*/

typedef struct {
    uint16_t TickNow;
    uint16_t TickLast;
    float UseTime;
    uint16_t TaskFreq;
    uint16_t LastFreq;
    uint8_t countState;
}RTOSDebugPrivateVar;

typedef struct {
    void (*TickStart)(RTOSDebugPrivateVar* priv_var);
    uint16_t (*UpdateFreq)(RTOSDebugPrivateVar* priv_var);
    float (*UpdateUsedTime)(RTOSDebugPrivateVar* priv_var);
}RtosDebugInterface_t,*pRTOSDebugInterface_t;

extern RtosDebugInterface_t RTOSDebugTimer;

#endif /* __BSP_RTOSDEBUG_H */

