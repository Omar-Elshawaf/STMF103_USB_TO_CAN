/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SLCAN (LAWICEL) USB-to-CAN Adapter Firmware
  *
  * Hardware:  STM32F103C8T6 "Blue Pill"
  * UART:      USART1 PA9(TX)/PA10(RX) @ 115200 -> USB-to-TTL module
  * CAN:       CAN1 remapped to PB8(RX)/PB9(TX) -> MCP2551 transceiver
  * Clock:     72 MHz from 8 MHz HSE crystal
  *
  * Protocol:  LAWICEL/SLCAN ASCII serial protocol for slcand compatibility
  *   O\r       - Open CAN channel
  *   C\r       - Close CAN channel
  *   S[0-8]\r  - Set CAN bitrate
  *   t[III][L][DD...]\r - Transmit standard CAN frame
  *   V\r       - Version number
  *   N\r       - Serial number
  *   F\r       - Read status flags
  *
  * RX frames are forwarded as: t[III][L][DD...]\r
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLCAN_MTU       32   /* Max command length: t + 3(ID) + 1(DLC) + 16(data) + \r + \0 */
#define SLCAN_OK        "\r"
#define SLCAN_ERR       "\a" /* BEL character = NACK per LAWICEL spec */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t  uart_rx_byte;              /* Single-byte interrupt RX target */
static char     slcan_buf[SLCAN_MTU];      /* Command accumulator             */
static uint8_t  slcan_idx = 0;             /* Current write position          */
static volatile uint8_t slcan_cmd_ready;   /* Flag: complete command received */
static char     slcan_cmd[SLCAN_MTU];      /* Snapshot of completed command   */
static uint8_t  can_is_open = 0;           /* CAN channel state               */

/* CAN bitrate table index set by 'S' command, applied on 'O' command.
 * Default S6 = 500 kbps. APB1 clock = 36 MHz on STM32F103 @ 72 MHz SYSCLK.
 *
 * Bit timing formula: Bitrate = APB1 / (Prescaler * (1 + BS1 + BS2))
 * All entries target ~87.5% sample point where possible.
 */
typedef struct {
  uint16_t prescaler;
  uint32_t bs1;
  uint32_t bs2;
} CAN_BitTiming_t;

static const CAN_BitTiming_t can_timings[9] = {
  /* S0:   10 kbps */ { 200, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(200*18)=10k   */
  /* S1:   20 kbps */ { 100, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(100*18)=20k   */
  /* S2:   50 kbps */ {  40, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(40*18) =50k   */
  /* S3:  100 kbps */ {  20, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(20*18) =100k  */
  /* S4:  125 kbps */ {  16, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(16*18) =125k  */
  /* S5:  250 kbps */ {   8, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(8*18)  =250k  */
  /* S6:  500 kbps */ {   4, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(4*18)  =500k  */
  /* S7:  800 kbps */ {   3, CAN_BS1_12TQ, CAN_BS2_2TQ },  /* 36M/(3*15)  =800k  */
  /* S8: 1000 kbps */ {   2, CAN_BS1_15TQ, CAN_BS2_2TQ },  /* 36M/(2*18)  =1000k */
};

static uint8_t can_speed_idx = 6; /* Default: 500 kbps */

/*---------------------------------------------------------------------------
 * CAN RX Watch List
 *
 * Commands:
 *   r[III]\r  — Register CAN ID III (3-hex-digit) for monitoring.
 *   R[III]\r  — Read latest data received on CAN ID III.
 *   u[III]\r  — Unregister CAN ID III, stop tracking it.
 *
 * Only frames whose ID is in the watch list get stored locally.
 * ALL frames are still forwarded via SLCAN text regardless.
 *---------------------------------------------------------------------------*/
#define CAN_WATCH_MAX  8   /* Max number of CAN IDs to watch simultaneously */

typedef struct {
  uint32_t id;                 /* Standard CAN ID (11-bit)          */
  uint8_t  dlc;                /* Data Length Code                  */
  uint8_t  data[8];            /* Payload bytes                     */
  volatile uint8_t fresh;      /* 1 = new data not yet consumed     */
  uint8_t  active;             /* 1 = slot is registered            */
} CAN_WatchEntry_t;

static CAN_WatchEntry_t can_watch[CAN_WATCH_MAX];
static uint8_t          can_watch_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void SLCAN_SendStr(const char *s);
static uint8_t SLCAN_HexCharToNibble(char c, uint8_t *out);
static char    SLCAN_NibbleToHexChar(uint8_t n);
static void SLCAN_ProcessCommand(const char *cmd);
static void SLCAN_OpenChannel(void);
static void SLCAN_CloseChannel(void);
static void SLCAN_TransmitFrame(const char *cmd);
static void SLCAN_RegisterWatch(const char *cmd);
static void SLCAN_ReadWatch(const char *cmd);
static void SLCAN_UnregisterWatch(const char *cmd);
static void SLCAN_ForwardRxFrame(CAN_RxHeaderTypeDef *hdr, uint8_t *data);
static void CAN_ConfigAcceptAllFilter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*===========================================================================
 * UART Helper
 *===========================================================================*/
static void SLCAN_SendStr(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 50);
}

/*===========================================================================
 * Hex Conversion Utilities
 *
 * ASCII hex character -> 4-bit nibble value
 * Returns 1 on success, 0 on invalid character.
 *===========================================================================*/
static uint8_t SLCAN_HexCharToNibble(char c, uint8_t *out)
{
  if (c >= '0' && c <= '9')      { *out = (uint8_t)(c - '0');       return 1; }
  else if (c >= 'a' && c <= 'f') { *out = (uint8_t)(c - 'a' + 10); return 1; }
  else if (c >= 'A' && c <= 'F') { *out = (uint8_t)(c - 'A' + 10); return 1; }
  return 0; /* Invalid hex character */
}

/* 4-bit nibble value -> uppercase ASCII hex character */
static char SLCAN_NibbleToHexChar(uint8_t n)
{
  static const char hex[] = "0123456789ABCDEF";
  return hex[n & 0x0F];
}

/*===========================================================================
 * CAN Accept-All Filter (Mask = 0x0000 passes everything)
 *===========================================================================*/
static void CAN_ConfigAcceptAllFilter(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = 0x0000;
  f.FilterIdLow          = 0x0000;
  f.FilterMaskIdHigh     = 0x0000;
  f.FilterMaskIdLow      = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &f);
}

/*===========================================================================
 * SLCAN Command: Open CAN Channel ('O')
 *===========================================================================*/
static void SLCAN_OpenChannel(void)
{
  if (can_is_open) { SLCAN_SendStr(SLCAN_ERR); return; }

  /* Apply the bitrate selected by the last 'S' command */
  hcan.Init.Prescaler    = can_timings[can_speed_idx].prescaler;
  hcan.Init.TimeSeg1     = can_timings[can_speed_idx].bs1;
  hcan.Init.TimeSeg2     = can_timings[can_speed_idx].bs2;
  hcan.Init.Mode         = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth     = CAN_SJW_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff        = DISABLE;
  hcan.Init.AutoWakeUp        = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked  = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK) { SLCAN_SendStr(SLCAN_ERR); return; }

  CAN_ConfigAcceptAllFilter();

  if (HAL_CAN_Start(&hcan) != HAL_OK) { SLCAN_SendStr(SLCAN_ERR); return; }

  /* Enable FIFO0 message pending interrupt */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  can_is_open = 1;
  SLCAN_SendStr(SLCAN_OK);
}

/*===========================================================================
 * SLCAN Command: Close CAN Channel ('C')
 *===========================================================================*/
static void SLCAN_CloseChannel(void)
{
  if (!can_is_open) { SLCAN_SendStr(SLCAN_ERR); return; }

  HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Stop(&hcan);
  can_is_open = 0;
  SLCAN_SendStr(SLCAN_OK);
}

/*===========================================================================
 * SLCAN Command: Transmit Standard CAN Frame ('t')
 *
 * Format: t[III][L][DD...]\0   (already stripped of trailing \r)
 *   III = 3 hex digits for 11-bit Standard ID (000-7FF)
 *   L   = 1 digit DLC (0-8)
 *   DD  = L pairs of hex digits for data bytes
 *
 * Example: "t1FF8DEADBEEFCAFEBABE" ->
 *   ID=0x1FF, DLC=8, Data={0xDE,0xAD,0xBE,0xEF,0xCA,0xFE,0xBA,0xBE}
 *===========================================================================*/
static void SLCAN_TransmitFrame(const char *cmd)
{
  if (!can_is_open) { SLCAN_SendStr(SLCAN_ERR); return; }

  /* Minimum length check: 't' + 3(ID) + 1(DLC) = 5 characters */
  uint8_t len = (uint8_t)strlen(cmd);
  if (len < 5) { SLCAN_SendStr(SLCAN_ERR); return; }

  /*--- Parse the 3-digit hex Standard ID from cmd[1..3] ---*/
  uint8_t n0, n1, n2;
  if (!SLCAN_HexCharToNibble(cmd[1], &n0) ||
      !SLCAN_HexCharToNibble(cmd[2], &n1) ||
      !SLCAN_HexCharToNibble(cmd[3], &n2))
  {
    SLCAN_SendStr(SLCAN_ERR); return;
  }
  uint32_t std_id = ((uint32_t)n0 << 8) | ((uint32_t)n1 << 4) | n2;
  if (std_id > 0x7FF) { SLCAN_SendStr(SLCAN_ERR); return; }

  /*--- Parse the 1-digit DLC from cmd[4] ---*/
  uint8_t dlc = (uint8_t)(cmd[4] - '0');
  if (dlc > 8) { SLCAN_SendStr(SLCAN_ERR); return; }

  /*--- Verify total length matches: 5 + (dlc * 2) ---*/
  if (len != (uint8_t)(5 + dlc * 2)) { SLCAN_SendStr(SLCAN_ERR); return; }

  /*--- Parse data bytes from cmd[5..] two hex chars each ---*/
  uint8_t data[8] = {0};
  for (uint8_t i = 0; i < dlc; i++) {
    uint8_t hi, lo;
    if (!SLCAN_HexCharToNibble(cmd[5 + i * 2],     &hi) ||
        !SLCAN_HexCharToNibble(cmd[5 + i * 2 + 1], &lo))
    {
      SLCAN_SendStr(SLCAN_ERR); return;
    }
    data[i] = (uint8_t)((hi << 4) | lo);
  }

  /*--- Build CAN TX header and transmit ---*/
  CAN_TxHeaderTypeDef txHeader;
  txHeader.StdId              = std_id;
  txHeader.ExtId              = 0;
  txHeader.IDE                = CAN_ID_STD;
  txHeader.RTR                = CAN_RTR_DATA;
  txHeader.DLC                = dlc;
  txHeader.TransmitGlobalTime = DISABLE;

  uint32_t txMailbox;
  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK) {
    SLCAN_SendStr(SLCAN_ERR);
    return;
  }

  /* 'Z\r' = successful transmit acknowledgement per LAWICEL spec */
  SLCAN_SendStr("Z\r");
}

/*===========================================================================
 * Convert a received CAN frame into LAWICEL text and send over UART
 *
 * Output format: t[III][L][DD...]\r
 * Example: ID=0x01F, DLC=3, Data={0xAB,0xCD,0xEF} -> "t01F3ABCDEF\r"
 *===========================================================================*/
static void SLCAN_ForwardRxFrame(CAN_RxHeaderTypeDef *hdr, uint8_t *data)
{
  char out[32];
  uint8_t pos = 0;

  if (hdr->IDE == CAN_ID_STD) {
    out[pos++] = 't';
    /* 3-digit hex Standard ID (zero-padded) */
    out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(hdr->StdId >> 8) & 0x07);
    out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(hdr->StdId >> 4) & 0x0F);
    out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(hdr->StdId)      & 0x0F);
  } else {
    /* Extended frame: 'T' + 8-digit hex Extended ID */
    out[pos++] = 'T';
    for (int8_t i = 7; i >= 0; i--) {
      out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(hdr->ExtId >> (i * 4)) & 0x0F);
    }
  }

  /* 1-digit DLC */
  out[pos++] = (char)('0' + (hdr->DLC & 0x0F));

  /* Data bytes as pairs of hex digits */
  for (uint8_t i = 0; i < hdr->DLC && i < 8; i++) {
    out[pos++] = SLCAN_NibbleToHexChar(data[i] >> 4);
    out[pos++] = SLCAN_NibbleToHexChar(data[i] & 0x0F);
  }

  out[pos++] = '\r';
  out[pos]   = '\0';

  SLCAN_SendStr(out);
}

/*===========================================================================
 * SLCAN Command: Register a CAN ID for local monitoring ('r')
 *
 * Format: r[III]\0   (3-digit hex Standard ID, already stripped of \r)
 * Example: "r009" -> watch CAN ID 0x009 (encoder on ODrive axis 0)
 *
 * Once registered, every frame with this ID that arrives on the bus will
 * be stored in can_watch[] so you can query it with 'R[III]'.
 * All frames are STILL forwarded via SLCAN text regardless.
 *===========================================================================*/
static void SLCAN_RegisterWatch(const char *cmd)
{
  /* Need exactly 4 chars: 'r' + 3 hex digits */
  if (strlen(cmd) != 4) { SLCAN_SendStr(SLCAN_ERR); return; }

  uint8_t n0, n1, n2;
  if (!SLCAN_HexCharToNibble(cmd[1], &n0) ||
      !SLCAN_HexCharToNibble(cmd[2], &n1) ||
      !SLCAN_HexCharToNibble(cmd[3], &n2))
  { SLCAN_SendStr(SLCAN_ERR); return; }

  uint32_t watch_id = ((uint32_t)n0 << 8) | ((uint32_t)n1 << 4) | n2;

  /* Check if already registered */
  for (uint8_t i = 0; i < CAN_WATCH_MAX; i++) {
    if (can_watch[i].active && can_watch[i].id == watch_id) {
      SLCAN_SendStr(SLCAN_OK); /* Already watching, just ACK */
      return;
    }
  }

  /* Find a free slot */
  if (can_watch_count >= CAN_WATCH_MAX) { SLCAN_SendStr(SLCAN_ERR); return; }

  for (uint8_t i = 0; i < CAN_WATCH_MAX; i++) {
    if (!can_watch[i].active) {
      can_watch[i].id     = watch_id;
      can_watch[i].dlc    = 0;
      can_watch[i].fresh  = 0;
      can_watch[i].active = 1;
      memset(can_watch[i].data, 0, 8);
      can_watch_count++;
      SLCAN_SendStr(SLCAN_OK);
      return;
    }
  }
  SLCAN_SendStr(SLCAN_ERR);
}

/*===========================================================================
 * SLCAN Command: Read latest data from a watched CAN ID ('R')
 *
 * Format: R[III]\0
 * Response on success: R[III][L][DD...]\r   (same layout as 't' frames)
 * Response if no data:  \a  (BEL / NACK)
 *
 * Example: send "R009" -> receive "R0098DEADBEEFCAFEBABE\r"
 *===========================================================================*/
static void SLCAN_ReadWatch(const char *cmd)
{
  if (strlen(cmd) != 4) { SLCAN_SendStr(SLCAN_ERR); return; }

  uint8_t n0, n1, n2;
  if (!SLCAN_HexCharToNibble(cmd[1], &n0) ||
      !SLCAN_HexCharToNibble(cmd[2], &n1) ||
      !SLCAN_HexCharToNibble(cmd[3], &n2))
  { SLCAN_SendStr(SLCAN_ERR); return; }

  uint32_t read_id = ((uint32_t)n0 << 8) | ((uint32_t)n1 << 4) | n2;

  /* Find the slot */
  for (uint8_t i = 0; i < CAN_WATCH_MAX; i++) {
    if (can_watch[i].active && can_watch[i].id == read_id) {
      if (!can_watch[i].fresh) {
        /* Registered but no data received yet */
        SLCAN_SendStr(SLCAN_ERR);
        return;
      }
      /* Build response: R[III][L][DD...]\r */
      char out[32];
      uint8_t pos = 0;
      out[pos++] = 'R';
      out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(read_id >> 8) & 0x07);
      out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(read_id >> 4) & 0x0F);
      out[pos++] = SLCAN_NibbleToHexChar((uint8_t)(read_id)      & 0x0F);
      out[pos++] = (char)('0' + can_watch[i].dlc);
      for (uint8_t j = 0; j < can_watch[i].dlc && j < 8; j++) {
        out[pos++] = SLCAN_NibbleToHexChar(can_watch[i].data[j] >> 4);
        out[pos++] = SLCAN_NibbleToHexChar(can_watch[i].data[j] & 0x0F);
      }
      out[pos++] = '\r';
      out[pos]   = '\0';
      can_watch[i].fresh = 0; /* Mark as consumed */
      SLCAN_SendStr(out);
      return;
    }
  }
  /* ID not in watch list */
  SLCAN_SendStr(SLCAN_ERR);
}

/*===========================================================================
 * SLCAN Command: Unregister a watched CAN ID ('u')
 *
 * Format: u[III]\0
 *===========================================================================*/
static void SLCAN_UnregisterWatch(const char *cmd)
{
  if (strlen(cmd) != 4) { SLCAN_SendStr(SLCAN_ERR); return; }

  uint8_t n0, n1, n2;
  if (!SLCAN_HexCharToNibble(cmd[1], &n0) ||
      !SLCAN_HexCharToNibble(cmd[2], &n1) ||
      !SLCAN_HexCharToNibble(cmd[3], &n2))
  { SLCAN_SendStr(SLCAN_ERR); return; }

  uint32_t unreg_id = ((uint32_t)n0 << 8) | ((uint32_t)n1 << 4) | n2;

  for (uint8_t i = 0; i < CAN_WATCH_MAX; i++) {
    if (can_watch[i].active && can_watch[i].id == unreg_id) {
      can_watch[i].active = 0;
      can_watch[i].fresh  = 0;
      can_watch_count--;
      SLCAN_SendStr(SLCAN_OK);
      return;
    }
  }
  SLCAN_SendStr(SLCAN_ERR); /* Was not registered */
}

/*===========================================================================
 * Master command dispatcher
 *===========================================================================*/
static void SLCAN_ProcessCommand(const char *cmd)
{
  if (cmd[0] == '\0') return;

  switch (cmd[0]) {
    case 'O':  /* Open CAN channel */
      SLCAN_OpenChannel();
      break;

    case 'C':  /* Close CAN channel */
      SLCAN_CloseChannel();
      break;

    case 'S':  /* Set CAN bitrate: S0..S8 */
      if (can_is_open) { SLCAN_SendStr(SLCAN_ERR); break; }
      if (cmd[1] >= '0' && cmd[1] <= '8' && cmd[2] == '\0') {
        can_speed_idx = (uint8_t)(cmd[1] - '0');
        SLCAN_SendStr(SLCAN_OK);
      } else {
        SLCAN_SendStr(SLCAN_ERR);
      }
      break;

    case 't':  /* Transmit standard CAN frame */
      SLCAN_TransmitFrame(cmd);
      break;

    case 'r':  /* Register CAN ID for local monitoring */
      SLCAN_RegisterWatch(cmd);
      break;

    case 'R':  /* Read latest data from a watched CAN ID */
      SLCAN_ReadWatch(cmd);
      break;

    case 'u':  /* Unregister a watched CAN ID */
      SLCAN_UnregisterWatch(cmd);
      break;

    case 'T':  /* Transmit extended CAN frame (not implemented) */
      SLCAN_SendStr(SLCAN_ERR);
      break;

    case 'V':  /* Hardware version */
      SLCAN_SendStr("V1010\r");
      break;

    case 'N':  /* Serial number */
      SLCAN_SendStr("NSTM32\r");
      break;

    case 'F':  /* Read status flags (return 0 = no errors) */
      SLCAN_SendStr("F00\r");
      break;

    default:   /* Unknown command */
      SLCAN_SendStr(SLCAN_ERR);
      break;
  }
}

/*===========================================================================
 * UART RX Complete Callback (interrupt-driven, 1 byte at a time)
 *
 * Accumulates characters into slcan_buf until '\r' is received.
 * On '\r', copies the buffer into slcan_cmd and sets the ready flag
 * for the main loop to process (avoids heavy work in ISR context).
 *===========================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) return;

  if (uart_rx_byte == '\r') {
    /* Command complete - null-terminate and snapshot */
    if (slcan_idx > 0 && !slcan_cmd_ready) {
      slcan_buf[slcan_idx] = '\0';
      memcpy(slcan_cmd, slcan_buf, slcan_idx + 1);
      slcan_cmd_ready = 1;
    }
    slcan_idx = 0;
  }
  else if (uart_rx_byte != '\n') {
    /* Accumulate character with overflow guard */
    if (slcan_idx < SLCAN_MTU - 1) {
      slcan_buf[slcan_idx++] = (char)uart_rx_byte;
    } else {
      /* Buffer overflow - discard this command */
      slcan_idx = 0;
    }
  }
  /* else: ignore \n characters */

  /* Re-arm the single-byte interrupt reception */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/*===========================================================================
 * CAN RX FIFO0 Message Pending Callback
 *
 * Called from ISR when a CAN frame arrives.  Two things happen:
 *   1) The frame is forwarded as LAWICEL text over UART (for slcand).
 *   2) If the frame's ID is in the watch list (registered via 'r[III]'),
 *      the data is stored locally so it can be queried with 'R[III]'.
 *===========================================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_irq)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t             rxData[8];

  if (HAL_CAN_GetRxMessage(hcan_irq, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    return;

  /* --- 1) Forward ALL frames to host as SLCAN text --- */
  SLCAN_ForwardRxFrame(&rxHeader, rxData);

  /* --- 2) Store only if this ID is in the watch list --- */
  if (can_watch_count == 0) return; /* Nothing registered, skip search */

  uint32_t rx_id = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;

  for (uint8_t i = 0; i < CAN_WATCH_MAX; i++) {
    if (can_watch[i].active && can_watch[i].id == rx_id) {
      can_watch[i].dlc = (uint8_t)rxHeader.DLC;
      for (uint8_t j = 0; j < rxHeader.DLC && j < 8; j++)
        can_watch[i].data[j] = rxData[j];
      can_watch[i].fresh = 1;
      break;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Kick off interrupt-driven single-byte UART reception */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Check if UART ISR has assembled a complete SLCAN command */
    if (slcan_cmd_ready) {
      SLCAN_ProcessCommand(slcan_cmd);
      slcan_cmd_ready = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSE 8 MHz -> PLL x9 -> 72 MHz SYSCLK
  *        APB1 = 36 MHz (CAN peripheral clock)
  *        APB2 = 72 MHz (USART1 peripheral clock)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  *        Default config: 500 kbps. Re-initialized on 'O' command
  *        with the bitrate selected by 'S' command.
  */
static void MX_CAN_Init(void)
{
  /* USER CODE BEGIN CAN_Init 0 */
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = can_timings[can_speed_idx].prescaler;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = can_timings[can_speed_idx].bs1;
  hcan.Init.TimeSeg2 = can_timings[can_speed_idx].bs2;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* NOTE: CAN is NOT started here. The 'O' command starts it. */
  /* USER CODE END CAN_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  *        115200 baud, 8N1, no flow control
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
