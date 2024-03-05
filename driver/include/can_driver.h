#ifndef __CAN_H__
#define __CAN_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Datatype Definiton
 ******************************************************************************/
typedef enum
{
    FLEXCAN_RETURN_CODE_SUCCESS = 0U,
    FLEXCAN_RETURN_CODE_FAIL,
    FLEXCAN_RETURN_CODE_INVALID_INS
} FlexCAN_ReturnCode_t;

typedef struct
{
    uint32_t timeStamp : 16;
    uint32_t dlc       : 4;
    uint32_t rtr       : 1;
    uint32_t ide       : 1;
    uint32_t srr       : 1;
    uint32_t reserve1  : 1;
    uint32_t code      : 4;
    uint32_t reserve2  : 1;
    uint32_t esi       : 1;
    uint32_t brs       : 1;
    uint32_t edl       : 1;
} FlexCAN_control_MB_t;

typedef struct
{
    uint32_t id   : 29;
    uint32_t prio : 3;
} FlexCAN_ID_config_MB_t;

typedef struct
{
    FlexCAN_control_MB_t cfControl;
    FlexCAN_ID_config_MB_t cfID;
    uint32_t RxIdMask;
} FlexCAN_RX_MessageBuffer_t;

typedef struct
{
    FlexCAN_control_MB_t cfControl;
    FlexCAN_ID_config_MB_t cfID;
    uint8_t dataByte[64];
} FlexCAN_TX_MessageBuffer_t;

typedef struct
{
    uint8_t presdiv;
    uint8_t rjw;
    uint8_t pseg1;
    uint8_t pseg2;
    uint8_t smp;
    uint8_t propseg;
} FlexCAN_bit_timing_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
typedef void (*FlexCAN_CallbackIRQ)(uint8_t);
FlexCAN_ReturnCode_t FlexCAN_Init(uint32_t instance, uint8_t wordSize, FlexCAN_bit_timing_t *bitTiming);
FlexCAN_ReturnCode_t FlexCAN_Config_Tx_MessageBuffer(uint32_t instance, uint8_t indexOfMB, FlexCAN_TX_MessageBuffer_t *DataOfMB);
FlexCAN_ReturnCode_t FlexCAN_Config_RX_MessageBuffer(uint32_t instance, uint8_t IndexOfMb, FlexCAN_RX_MessageBuffer_t *config);
FlexCAN_ReturnCode_t FlexCAN_Send(uint32_t instance, uint8_t IndexOfMb, FlexCAN_TX_MessageBuffer_t *mbData);
FlexCAN_ReturnCode_t FlexCAN_Receive(uint32_t instance, uint8_t IndexOfMb, FlexCAN_TX_MessageBuffer_t *mbData);
FlexCAN_ReturnCode_t FlexCAN_ConfigInterrupt(uint32_t instance, uint8_t IndexOfMb);
FlexCAN_ReturnCode_t FlexCAN_InitIRQ(uint32_t instance, IRQn_Type irqIndex, FlexCAN_CallbackIRQ CAN_MiddlewareCallback);
FlexCAN_ReturnCode_t FlexCAN_ClearInterruptFlag(uint32_t instance, uint32_t flagIndex);

#endif /* __CAN_H__ */
/*******************************************************************************
 * End of file
 ******************************************************************************/
