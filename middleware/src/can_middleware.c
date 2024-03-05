/*******************************************************************************
 * Include
 ******************************************************************************/
#include "can_middleware.h"
#
/*******************************************************************************
 * Macros
 ******************************************************************************/
#define CAN_0 (0U)
#define CAN_1 (1U)
#define CAN_2 (2U)

#define ONE_BYTE (8U)
#define TWO_BYTES (16U)
#define THREE_BYTES (24U)

#define OFFSET_STANDARD_ID_MB (18U)
#define MSG_BUF_WORD_SIZE (4u)

#define MB_TRANSMIT_INDEX (0U)
#define MB_RECEIVE_INDEX (1U)
#define MB_MAX_DLC (8U)

#define ID_FORWARDER_DISTANCE (0U)
#define ID_FORWARDER_ANGEL (1U)

#define UART_LENGTH (12U)
#define UART_SOF (0x53U)
#define UART_EOF (0x45U)

/*******************************************************************************
 * Variables Definition
 ******************************************************************************/
/* Data to config bit timing */
static FlexCAN_bit_timing_t s_bitTiming =
{
    .propseg = 6u,
    .pseg1 = 3u,
    .pseg2 = 3u,
    .rjw = 3u,
    .presdiv = 0u,
    .smp = 1u
};
/* Data config control and status word MB */
static FlexCAN_control_MB_t s_config =
{
    .brs = 0U,
    .srr = 1U,
    .edl = 0U,
    .rtr = 0U,
    .esi = 0U,
    .ide = 1U
};

static Node_Config_t *s_NodeConfigPtr;
static uint8_t s_txMsgBuffer[12];
static CAN_Queue_Struct_t s_queueCanTransmit;
static CAN_Queue_Struct_t s_queueCanReceive;
static CAN_Middleware_TxCallback s_callbackTransmit = NULL;
static CAN_Middleware_RxCallback s_callbackReceive = NULL;

/*******************************************************************************
 * Prototype
 ******************************************************************************/
static void CANMiddleWare_ConvertDataUartToCan(FlexCAN_TX_MessageBuffer_t *messageBuffer, uint8_t *data);
static void CANMiddleware_IrqHandler(uint8_t flagInterruptMB);
static void CANMiddleWare_CreateMessageBuffer(FlexCAN_TX_MessageBuffer_t *msgBuffer, uint32_t data, CAN_Middleware_FrameTypes_t frameType);
/*******************************************************************************
 * Function
 ******************************************************************************/
/** @brief: UART frame
 * byte 0: SOF
 * byte 1: Data Length
 * byte 2: Node Type
 * byte 3: Frame Type
 * byte 4-5: Node ID
 * byte 6-8: Data
 * byte 9: Step Data
 * byte 10: Check Sum
 * byte 11: EOF
**/

/** @brief CAN frame 
 * byte 0: node type
 * byte 1: frame type
 * byte 2-3: node id
 * byte 4-6: data
 * byte 7: threshold
**/

static void CANMiddleWare_ConvertDataUartToCan(FlexCAN_TX_MessageBuffer_t *messageBuffer, uint8_t *data)
{
    uint8_t index;

    if (NULL != data)
    {
        messageBuffer->cfControl.srr = 1U;
        messageBuffer->cfControl.ide = 1U;
        messageBuffer->cfControl.edl = 0U;
        messageBuffer->cfControl.brs = 0U;
        messageBuffer->cfControl.esi = 0U;
        messageBuffer->cfControl.rtr = 0U;
        messageBuffer->cfControl.dlc = MB_MAX_DLC;
        messageBuffer->cfID.id = 0U;
        messageBuffer->cfID.id = (uint32_t)((((data[4]) << ONE_BYTE) | (data[5])) << OFFSET_STANDARD_ID_MB);
        messageBuffer->cfID.prio = 0U;
        for (index = 0; index < messageBuffer->cfControl.dlc; index++)
        {
            messageBuffer->dataByte[index] = data[2 + index];
        }
    }
}

static void CANMiddleware_IrqHandler(uint8_t flagInterruptMB)
{
    FlexCAN_TX_MessageBuffer_t *msgTXBuff = NULL;
    FlexCAN_TX_MessageBuffer_t msgRXBuff;

    if (flagInterruptMB == MB_TRANSMIT_INDEX)
    {
        FlexCAN_ClearInterruptFlag(CAN_0, MB_TRANSMIT_INDEX);
        CAN_Queue_Pop(&s_queueCanTransmit);
        CAN_Queue_Peek(&s_queueCanTransmit, &msgTXBuff);
        if (msgTXBuff != NULL)
        {
            FlexCAN_Send(CAN_0, MB_TRANSMIT_INDEX, msgTXBuff);
        }
        if(s_callbackTransmit != NULL) 
        {
        	s_callbackTransmit();
        }
    }
    if (flagInterruptMB == MB_RECEIVE_INDEX)
    {
        FlexCAN_ClearInterruptFlag(CAN_0, MB_RECEIVE_INDEX);
        FlexCAN_Receive(CAN_0, MB_RECEIVE_INDEX, &msgRXBuff);
        CAN_Queue_Push(&s_queueCanReceive, &msgRXBuff);
        if(s_callbackReceive != NULL)
        {
        	s_callbackReceive();
        }
    }
}

static void CANMiddleWare_CreateMessageBuffer(FlexCAN_TX_MessageBuffer_t *msgBuffer, uint32_t data, CAN_Middleware_FrameTypes_t frameType)
{
    msgBuffer->cfControl = s_config;
    if(NODE_TYPE_DISTANCE == s_NodeConfigPtr->nodeType)
    {
        msgBuffer->cfID.id = (uint32_t)ID_FORWARDER_DISTANCE; /* 0 */
    }
    else
    {
        msgBuffer->cfID.id = (uint32_t)(ID_FORWARDER_ANGEL << OFFSET_STANDARD_ID_MB); /* 1 */
    }
    msgBuffer->cfControl.dlc = MB_MAX_DLC;
    msgBuffer->dataByte[0] = (uint8_t)(s_NodeConfigPtr->nodeType);
    msgBuffer->dataByte[1] = (uint8_t)frameType;
    msgBuffer->dataByte[2] = (uint8_t)(s_NodeConfigPtr->nodeID);
    msgBuffer->dataByte[3] = (uint8_t)(s_NodeConfigPtr->nodeID >> ONE_BYTE);
    msgBuffer->dataByte[4] = (uint8_t)((data & 0xFF0000) >> TWO_BYTES);
    msgBuffer->dataByte[5] = (uint8_t)((data & 0xFF00) >> ONE_BYTE);
    msgBuffer->dataByte[6] = (uint8_t)(data & 0xFF);
    msgBuffer->dataByte[7] = (uint8_t)(s_NodeConfigPtr->threshold);
}

/* Can middleware for Forwarder */
void CANMiddleWare_ConvertDataCanToUart(uint8_t **data)
{
    uint8_t index = 0;
    uint8_t checkSum = 0;
    FlexCAN_TX_MessageBuffer_t *rxMsgBuffer = NULL;

    if (NULL != data)
    {
        CAN_Queue_Peek(&s_queueCanReceive, &rxMsgBuffer);
        CAN_Queue_Pop(&s_queueCanReceive);
        if (NULL != rxMsgBuffer)
        {
            s_txMsgBuffer[0] = UART_SOF;
            s_txMsgBuffer[1] = UART_LENGTH;
            s_txMsgBuffer[2] = rxMsgBuffer->dataByte[0]; /* node type */
            s_txMsgBuffer[3] = rxMsgBuffer->dataByte[1]; /* frame type */
            s_txMsgBuffer[4] = rxMsgBuffer->dataByte[3]; /* node id */
            s_txMsgBuffer[5] = rxMsgBuffer->dataByte[2]; /* node id */
            for (index = 6; index < 9; index++)
            {
                s_txMsgBuffer[index] = rxMsgBuffer->dataByte[index-2]; /* data */
            }
            s_txMsgBuffer[9] = rxMsgBuffer->dataByte[7]; /* threshold */
            s_txMsgBuffer[11] = UART_EOF;
            /* checksum */
            checkSum += UART_EOF + UART_SOF + UART_LENGTH;
            for (index = 2; index < UART_LENGTH - 2; index++)
            {
                checkSum += s_txMsgBuffer[index];
            }
            s_txMsgBuffer[10] = (uint8_t)(0xFFU - checkSum);
            *data = (uint8_t *)&s_txMsgBuffer[0];
        }
    }
}

/* Can middleware send  for Forwarder */
void CANMiddlewareFwd_TransmitData(uint8_t *data)
{
    FlexCAN_TX_MessageBuffer_t msgBuff;

    CANMiddleWare_ConvertDataUartToCan(&msgBuff, data);
    CAN_Queue_Push(&s_queueCanTransmit, &msgBuff);
    if (s_queueCanTransmit.size == 1u)
    {
        FlexCAN_Send(CAN_0, MB_TRANSMIT_INDEX, &msgBuff);
    }
}

void CANMiddlewareNode_TransmitData(uint32_t data, CAN_Middleware_FrameTypes_t frameType)
{
    FlexCAN_TX_MessageBuffer_t msgBuff;

    CANMiddleWare_CreateMessageBuffer(&msgBuff, data, frameType);
    CAN_Queue_Push(&s_queueCanTransmit, &msgBuff);
    if (s_queueCanTransmit.size == 1)
    {
        FlexCAN_Send(CAN_0, MB_TRANSMIT_INDEX, &msgBuff);
    }
}

CAN_Middleware_FrameTypes_t CANMiddlewareNode_CheckRequest(Node_Config_t *nodeConfig)
{
    FlexCAN_TX_MessageBuffer_t *dataReceive = NULL;
    CAN_Middleware_FrameTypes_t retVal = FRAME_TYPE_CONFIG;

    CAN_Queue_Peek(&s_queueCanReceive, &dataReceive);
    CAN_Queue_Pop(&s_queueCanReceive);
    if (dataReceive != NULL)
    {
        /* Get request type */
        retVal = (dataReceive->dataByte[1]);

        if (retVal == FRAME_TYPE_CONFIG)
        {
            nodeConfig->nodeID = (uint16_t)(dataReceive->dataByte[6]) | (uint16_t)((dataReceive->dataByte[5]) << ONE_BYTE);
            nodeConfig->nodeType = dataReceive->dataByte[0];
            nodeConfig->threshold = dataReceive->dataByte[7];
        }
    }

    return retVal;
}

void CANMiddleware_Init(CAN_MiddlewareConfig_t *config)
{
    FlexCAN_RX_MessageBuffer_t configMbRx;

    /* CAN0: RX -> PTE4 */
    PORTE->PCR[4] &= (~(PORT_PCR_MUX_MASK));
    PORTE->PCR[4] |= PORT_PCR_MUX(5);
    /* CAN0: TX -> PTE5 */
    PORTE->PCR[5] &= (~(PORT_PCR_MUX_MASK));
    PORTE->PCR[5] |= PORT_PCR_MUX(5);
    s_callbackTransmit = config->TxCallback;
    s_callbackReceive = config->RxCallback;
    s_NodeConfigPtr = config->nodeConfigPtr;
    configMbRx.cfControl = s_config;
    
    /**** Config ID for CAN for fwd ****/
    if ((s_NodeConfigPtr->nodeType == NODE_TYPE_FORWARDER) \
    || (s_NodeConfigPtr->nodeType == NODE_TYPE_ANGLE)      \
    || (s_NodeConfigPtr->nodeType == NODE_TYPE_DISTANCE))
    {
        configMbRx.RxIdMask = 0;
        configMbRx.RxIdMask = (s_NodeConfigPtr->nodeID) << OFFSET_STANDARD_ID_MB;
        configMbRx.cfID.id  = 0;
        configMbRx.cfID.id  = (s_NodeConfigPtr->nodeID) << OFFSET_STANDARD_ID_MB;
    }
  
    FlexCAN_Init(CAN_0, MSG_BUF_WORD_SIZE, &s_bitTiming);
    FlexCAN_Config_RX_MessageBuffer(CAN_0, MB_RECEIVE_INDEX, &configMbRx);
    /* enable interrupt */
    FlexCAN_ConfigInterrupt(CAN_0, MB_TRANSMIT_INDEX);
    FlexCAN_ConfigInterrupt(CAN_0, MB_RECEIVE_INDEX);
    FlexCAN_InitIRQ(CAN_0, CAN0_ORed_0_15_MB_IRQn, CANMiddleware_IrqHandler);
}

/*******************************************************************************
 * End of file
 ******************************************************************************/
