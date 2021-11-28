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
 */
#ifndef STM32_CAN_LIGHT_H_INCLUDED
#define STM32_CAN_LIGHT_H_INCLUDED

#include "my_fp.h"

#define CAN_ERR_INVALID_ID -1
#define CAN_ERR_INVALID_OFS -2
#define CAN_ERR_INVALID_LEN -3
#define CAN_ERR_MAXMESSAGES -4
#define CAN_ERR_MAXITEMS -5

class CANIDMAP;
class SENDBUFFER;

class Can
{
public:
   enum baudrates
   {
      Baud250, Baud500, Baud800, Baud1000, BaudLast
   };

   Can(uint32_t baseAddr, enum baudrates baudrate);
   void Clear(void);
   void SetBaudrate(enum baudrates baudrate);
   void Send(uint32_t canId, uint32_t data[2]) { Send(canId, data, 8); }
   void Send(uint32_t canId, uint32_t data[2], uint8_t len);
   void SendAll();
   void Save();
   void SetReceiveCallback(void (*recv)(uint32_t, uint32_t*,uint8_t));
   bool RegisterUserMessage(int canId);
   uint32_t GetLastRxTimestamp();  
   void HandleRx(int fifo);
   void HandleTx();
   static Can* GetInterface(int index);

private:
   static const int MAX_ITEMS_PER_MESSAGE = 8;
   static const int MAX_MESSAGES = 10;
   static const int SENDBUFFER_LEN = 20;
   static const int MAX_USER_MESSAGES = 10;

   struct CANPOS
   {
      uint16_t mapParam;
      s16fp gain;
      uint8_t offsetBits;
      int8_t numBits;
   };

   struct CANIDMAP
   {
      uint16_t canId;
      CANPOS items[MAX_ITEMS_PER_MESSAGE];
   };

   struct SENDBUFFER
   {
      uint16_t id;
      uint32_t len;
      uint32_t data[2];
   };

   uint32_t lastRxTimestamp;
   uint32_t canDev = CAN1;

   SENDBUFFER sendBuffer[SENDBUFFER_LEN];
   uint16_t userIds[MAX_USER_MESSAGES];

   int sendCnt;
   void (*recvCallback)(uint32_t, uint32_t*,uint8_t);
   
   int nextUserMessageIndex=0;
      
   void ConfigureFilters();
   void SetFilterBank(int& idIndex, int& filterId, uint16_t* idList);

   static Can* interfaces[];
};


#endif
