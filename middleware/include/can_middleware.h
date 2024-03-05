#ifndef __CAN_MIDDLEWARE_H__
#define __CAN_MIDDLEWARE_H__


/*******************************************************************************
 * Include
 ******************************************************************************/
#include "can_driver.h"
#include "queue_can.h"
#include "types_common.h"
/*******************************************************************************
 * Datatype Definiton
 ******************************************************************************/

typedef enum CAN__Middleware_FrameTypes_t
{
    FRAME_TYPE_CONFIG                    = 1U,
    FRAME_TYPE_CHECK_CONNECTION          = 2U,
    FRAME_TYPE_READ_DATA                 = 3U,
    FRAME_TYPE_RESET                     = 4U,
    FRAME_TYPE_CONFIG_RESPONSE           = 5U,
    FRAME_TYPE_CHECK_CONNECTION_RESPONSE = 6U,
    FRAME_TYPE_READ_DATA_RESPONSE        = 7U,
    FRAME_TYPE_RESET_RESPONSE            = 8U        
} CAN_Middleware_FrameTypes_t;

typedef void (*CAN_Middleware_TxCallback)(void);
typedef void (*CAN_Middleware_RxCallback)(void);

typedef struct CAN_MiddlewareConfig_t
{
    CAN_Middleware_TxCallback TxCallback;
    CAN_Middleware_RxCallback RxCallback;
    Node_Config_t *nodeConfigPtr;
} CAN_MiddlewareConfig_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
void CANMiddleWare_ConvertDataCanToUart(uint8_t **data);
void CANMiddlewareFwd_TransmitData(uint8_t *data);
void CANMiddlewareNode_TransmitData(uint32_t data, CAN_Middleware_FrameTypes_t frameType);
CAN_Middleware_FrameTypes_t CANMiddlewareNode_CheckRequest(Node_Config_t *nodeConfig);
void CANMiddleware_Init(CAN_MiddlewareConfig_t *config);

/*******************************************************************************
 * End of file
 ******************************************************************************/

#endif /* __CAN_MIDDLEWARE_H__ */
