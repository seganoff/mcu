#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#define CFG_TUSB_MCU                OPT_MCU_STM32F4

#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED             1
#define CFG_TUD_CDC                 1

#define CFG_TUD_ENDPOINT0_SIZE     64

#define CFG_TUD_CDC_RX_BUFSIZE     64
#define CFG_TUD_CDC_TX_BUFSIZE     64

#define CFG_TUD_DWC2_SLAVE_ENABLE   1
#define CFG_TUD_DWC2_DMA_ENABLE     0
#endif

