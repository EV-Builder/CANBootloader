/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 * EV_Builder: I tried to make this class as light as possible for bootloader usage;
 * ToDo: Point from the normal can libary to this one so it includes automaticly??
 * 
 * 
 */
#include <stdint.h>
#include "my_string.h"
#include "my_math.h"
#include "printf.h"
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include "stm32_can_light.h"

#define MAX_INTERFACES        2
#define IDS_PER_BANK          4

// #define SDO_WRITE             0x40
// #define SDO_READ              0x22
// #define SDO_ABORT             0x80
// #define SDO_WRITE_REPLY       0x23
// #define SDO_READ_REPLY        0x43
// #define SDO_ERR_INVIDX        0x06020000
// #define SDO_ERR_RANGE         0x06090030
// #define SENDMAP_ADDRESS       CANMAP_ADDRESS
// #define RECVMAP_ADDRESS       (CANMAP_ADDRESS + sizeof(canSendMap))
// #define CRC_ADDRESS           (CANMAP_ADDRESS + sizeof(canSendMap) + sizeof(canRecvMap))
// #define SENDMAP_WORDS         (sizeof(canSendMap) / sizeof(uint32_t))
// #define RECVMAP_WORDS         (sizeof(canRecvMap) / sizeof(uint32_t))
// #define CANID_UNSET           0xffff
// #define NUMBITS_LASTMARKER    -1
// #define forEachCanMap(c,m) for (CANIDMAP *c = m; (c - m) < MAX_MESSAGES && c->canId < CANID_UNSET; c++)
// #define forEachPosMap(c,m) for (CANPOS *c = m->items; (c - m->items) < MAX_ITEMS_PER_MESSAGE && c->numBits > 0; c++)

// #if (2 *((MAX_ITEMS_PER_MESSAGE * 6 + 2) * MAX_MESSAGES + 2) + 4) > FLASH_PAGE_SIZE
// #error CANMAP will not fit in one flash page
// #endif

// struct CAN_SDO
// {
//    uint8_t cmd;
//    uint16_t index;
//    uint8_t subIndex;
//    uint32_t data;
// } __attribute__((packed));

struct CANSPEED
{
   uint32_t ts1;
   uint32_t ts2;
   uint32_t prescaler;
};

Can* Can::interfaces[MAX_INTERFACES];

static void DummyCallback(uint32_t i, uint32_t* d,uint8_t l) { i=i; d=d;l=l;}


//ToDo: add a define or setting to use both tables???
//CAN TABLE FOR 72MHZ//36MHZ CAN HW BUS  SPEED.
// +/-60% Sample point
// static const CANSPEED canSpeed[Can::BaudLast] =
// {
//    { CAN_BTR_TS1_9TQ, CAN_BTR_TS2_6TQ, 9 }, //250kbps 62.5%
//    { CAN_BTR_TS1_4TQ, CAN_BTR_TS2_3TQ, 9 }, //500kbps 62.5%
//    { CAN_BTR_TS1_8TQ, CAN_BTR_TS2_6TQ, 3 }, //800kbps 60.0%
//    { CAN_BTR_TS1_10TQ, CAN_BTR_TS2_7TQ, 2 }, //1000kbps 61.1%
// };

//CAN TABLE FOR 64HZ//32MHZ CAN HW BUS  SPEED
// +/-70% SAMPLE POINT see: http://www.bittiming.can-wiki.info/
static const CANSPEED canSpeed[Can::BaudLast] =
 {
    { CAN_BTR_TS1_10TQ, CAN_BTR_TS2_5TQ, 8 }, //250kbps 68.8%
    { CAN_BTR_TS1_10TQ, CAN_BTR_TS2_5TQ, 4 }, //500kbps 68.8%
    { CAN_BTR_TS1_13TQ, CAN_BTR_TS2_6TQ, 2 }, //800kbps  70.0%
    { CAN_BTR_TS1_10TQ, CAN_BTR_TS2_5TQ, 2 }, //1000kbps  68.8%
 };



/** \brief Set function to be called for user handled CAN messages
 *
 * \param recv Function pointer to void func(uint32_t, uint32_t[2]) - ID, Data
 */
void Can::SetReceiveCallback(void (*recv)(uint32_t, uint32_t*,uint8_t))
{
   recvCallback = recv;
}


void Can::ConfigureFilters()
{
   uint16_t idList[IDS_PER_BANK] = { 0, 0, 0, 0 };
   int idIndex = 0;
   int filterId = canDev == CAN1 ? 0 : ((CAN_FMR(CAN2) >> 8) & 0x3F);

   for (int i = 0; i < nextUserMessageIndex; i++)
   {
      idList[idIndex] = userIds[i];
      idIndex++;

      if (idIndex == IDS_PER_BANK)
      {
         SetFilterBank(idIndex, filterId, idList);
      }
   }
   
   //loop terminates before adding last set of filters
   if (idIndex > 0)
   {
      SetFilterBank(idIndex, filterId, idList);
   }
}

/** \brief Add CAN Id to user message list
 * \post Receive callback will be called when a message with this Id id received
 * \param canId CAN identifier of message to be user handled
 * \return true: success, false: already 10 messages registered
 *
 */
bool Can::RegisterUserMessage(int canId)
{
   if (nextUserMessageIndex < MAX_USER_MESSAGES)
   {
      userIds[nextUserMessageIndex] = canId;
      nextUserMessageIndex++;
      ConfigureFilters();
      return true;
   }
   return false;
}

/** \brief Clear all defined messages
 */
void Can::Clear()
{
   //We should clear here the filters?
}


/** \brief Init can hardware with given baud rate
 * Initializes the following sub systems:
 * - CAN hardware itself
 * - Appropriate GPIO pins (non-remapped)
 * - Enables appropriate interrupts in NVIC
 *
 * \param baseAddr base address of CAN peripheral, CAN1 or CAN2
 * \param baudrate enum baudrates
 * \return void
 *
 */
Can::Can(uint32_t baseAddr, enum baudrates baudrate)
   : lastRxTimestamp(0), sendCnt(0), recvCallback(DummyCallback), nextUserMessageIndex(0), canDev(baseAddr)
{
   Clear();
   
   switch (baseAddr)
   {
      case CAN1:
         // Configure CAN pin: RX (input pull-up).
         gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
         gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);
         // Configure CAN pin: TX.-
         gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);
         //CAN1 RX and TX IRQs
         nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
         nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN_RX1_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN_RX1_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
         nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf << 4); //lowest priority
         interfaces[0] = this;
         break;
      case CAN2:

         gpio_set_mode(GPIO_BANK_CAN2_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX);
         gpio_set(GPIO_BANK_CAN2_RX, GPIO_CAN2_RX);
         gpio_set_mode(GPIO_BANK_CAN2_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX);

         //CAN2 RX and TX IRQs
         nvic_enable_irq(NVIC_CAN2_RX0_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_RX0_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN2_RX1_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_RX1_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN2_TX_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_TX_IRQ, 0xf << 4); //lowest priority
         interfaces[1] = this;
         break;
   }

	// Reset CAN
	can_reset(canDev);

	SetBaudrate(baudrate);
   
	// Enable CAN RX interrupts.
	can_enable_irq(canDev, CAN_IER_FMPIE0);
	can_enable_irq(canDev, CAN_IER_FMPIE1);
}

/** \brief Set baud rate to given value
 *
 * \param baudrate enum baudrates
 * \return void
 *
 */
void Can::SetBaudrate(enum baudrates baudrate)
{
	// CAN cell init.
	 // Setting the bitrate to 250KBit. APB1 = 36MHz,
	 // prescaler = 9 -> 4MHz time quanta frequency.
	 // 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
	 // 16time quanto per bit period, therefor 4MHz/16 = 250kHz
	 //
	can_init(canDev,
		     false,          // TTCM: Time triggered comm mode?
		     true,           // ABOM: Automatic bus-off management?
		     false,          // AWUM: Automatic wakeup mode?
		     false,          // NART: No automatic retransmission?
		     false,          // RFLM: Receive FIFO locked mode?
		     false,          // TXFP: Transmit FIFO priority?
		     CAN_BTR_SJW_1TQ,
		     canSpeed[baudrate].ts1,
		     canSpeed[baudrate].ts2,
		     canSpeed[baudrate].prescaler,				// BRP+1: Baud rate prescaler
		     false,
		     false);
}

/** \brief Get RTC time when last message was received
 *
 * \return uint32_t RTC time
 *
 */
uint32_t Can::GetLastRxTimestamp()
{
   return lastRxTimestamp;
}

/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
void Can::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
   can_disable_irq(canDev, CAN_IER_TMEIE);

   if (can_transmit(canDev, canId, false, false, len, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
   {
      /* enqueue in send buffer if all TX mailboxes are full */
      sendBuffer[sendCnt].id = canId;
      sendBuffer[sendCnt].len = len;
      sendBuffer[sendCnt].data[0] = data[0];
      sendBuffer[sendCnt].data[1] = data[1];
      sendCnt++;
   }

   if (sendCnt > 0)
   {
      can_enable_irq(canDev, CAN_IER_TMEIE);
   }
}

Can* Can::GetInterface(int index)
{
   if (index < MAX_INTERFACES)
   {
      return interfaces[index];
   }
   return 0;
}

void Can::HandleRx(int fifo)
{
   uint32_t id;
	bool ext, rtr;
	uint8_t length, fmi;
	uint32_t data[2];
      
   can_receive(canDev, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, NULL);

   recvCallback(id, data,length);

}

void Can::HandleTx()
{
   while (sendCnt > 0 && can_transmit(canDev, sendBuffer[sendCnt - 1].id, false, false, sendBuffer[sendCnt - 1].len, (uint8_t*)sendBuffer[sendCnt - 1].data) >= 0)
      sendCnt--;

   if (sendCnt == 0)
   {
      can_disable_irq(canDev, CAN_IER_TMEIE);
   }
}

void Can::SetFilterBank(int& idIndex, int& filterId, uint16_t* idList)
{
   can_filter_id_list_16bit_init(
         filterId,
         idList[0] << 5, //left align
         idList[1] << 5,
         idList[2] << 5,
         idList[3] << 5,
         filterId & 1,
         true);
   idIndex = 0;
   filterId++;
   idList[0] = idList[1] = idList[2] = idList[3] = 0;
}


/* Interrupt service routines */
extern "C" void usb_lp_can_rx0_isr(void)
{
   Can::GetInterface(0)->HandleRx(0);
}

extern "C" void can_rx1_isr()
{
   Can::GetInterface(0)->HandleRx(1);
}

extern "C" void usb_hp_can_tx_isr()
{
   Can::GetInterface(0)->HandleTx();
}

extern "C" void can2_rx0_isr()
{
   Can::GetInterface(1)->HandleRx(0);
}

extern "C" void can2_rx1_isr()
{
   Can::GetInterface(1)->HandleRx(1);
}

extern "C" void can2_tx_isr()
{
   Can::GetInterface(1)->HandleTx();
}
