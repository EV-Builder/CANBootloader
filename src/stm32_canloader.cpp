/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include "stm32_canloader.h"

#define FLASH_START 0x08000000
#define SMALLEST_PAGE_WORDS 256
#define PROGRAM_WORDS       512
#define APP_FLASH_START 0x08004000 //should be the correct value for the program...
#define BOOTLOADER_MAGIC 0xAA
#define DELAY_100 (1 << 20)
#define DELAY_200 (1 << 21)

#define nodeCANID 0x7DE
#define masterCANID 0x7DD

static Can* canobj1;
CanMsg txmsg;

uint8_t gbstate=0;

uint8_t magic = 0; 
uint8_t numPages = 0;
uint8_t actpage  = 0;
uint32_t bytecount = 0;
uint32_t startbytereq = 0;
int32_t bytesleft=0;

uint32_t recvCrc = 0;

const uint32_t receiveWords = SMALLEST_PAGE_WORDS;
uint32_t page_buffer[PROGRAM_WORDS];
uint32_t pgwrite=0;

uint32_t addr = APP_FLASH_START;
uint32_t bufferOffset = 0;

void initbuffer(){
   pgwrite=0;
   bufferOffset=0;

   for (int i = 0; i < PROGRAM_WORDS; i++)
      page_buffer[i]=0;

}

//Check 1k of flash whether it contains only 0xFF = erased
static bool check_erased(uint32_t* baseAddress)
{
   uint32_t check = 0xFFFFFFFF;

   for (int i = 0; i < SMALLEST_PAGE_WORDS; i++, baseAddress++)
      check &= *baseAddress;

   return check == 0xFFFFFFFF;
}

//We always write 2kb pages. After erasing the possible first page we check the
//data content of the possible second page. If it is not erased, it will be.
static void write_flash(uint32_t addr, uint32_t *pageBuffer)
{
   flash_erase_page(addr);

   if (!check_erased(((uint32_t*)addr) + SMALLEST_PAGE_WORDS))
      flash_erase_page(addr + SMALLEST_PAGE_WORDS * 4);

   for (uint32_t idx = 0; idx < PROGRAM_WORDS; idx++)
   {
      flash_program_word(addr + idx * 4, pageBuffer[idx]);
   }
}

void wait(void)
{
   for (volatile uint32_t i = DELAY_100; i > 0; i--);
}

void canSend(CanMsg *msg){
	   
   canobj1->Send(msg->id, (uint32_t*)msg->data,msg->len);

}

void SendDataRequest(uint32_t len){
         
         if (len>8) len=8;

         txmsg.id       = nodeCANID;
         txmsg.len      = 8;
         txmsg.data[0]  = 'P';
         txmsg.data[1]  = actpage;
         txmsg.data[2]  = numPages;
         txmsg.data[3]  = len & 0xFF;
         txmsg.data[4]  = (startbytereq & 0xFF);
         txmsg.data[5]  = ((startbytereq >> 8) & 0xFF);
         txmsg.data[6]  = ((startbytereq >> 16) & 0xFF);
         txmsg.data[7]  = ((startbytereq >> 24) & 0xFF);

         canSend(&txmsg);
}

void SendCRCRequest(){
         
         txmsg.len=8;
         txmsg.data[0]  = 'C';        
         txmsg.data[1]  = (actpage & 0xFF);
         txmsg.data[2]  = (numPages & 0xFF);
         txmsg.data[3]  = (bufferOffset & 0xFF);
         txmsg.data[4]  = (pgwrite & 0xFF);
         txmsg.data[5]  = ((pgwrite >> 8) & 0xFF);
         txmsg.data[6]  = ((pgwrite >> 16) & 0xFF);
         txmsg.data[7]  = ((pgwrite >> 24) & 0xFF);
    
         canSend(&txmsg);
}


static void CanCallback(uint32_t id, uint32_t data[2], uint8_t len) //This is where we go when a defined CAN message is received.
{

   uint8_t *pdata; //a pointer to point to the first byte in the data section
   
   //grab the adress so we have a byte pointer pointing to the 8 bytes...
   pdata = (uint8_t*)&data[0];

    switch (id)
    {
      case masterCANID:

             switch (gbstate){
               case 0:
                     magic = *pdata; //only one byte
                     //reinit these values...
                     pgwrite=0;
                     numPages=0;
                     bytecount=0;
                     gbstate=1;
                 break;
               case 1:                     
                     numPages = *pdata; //only one byte
                     bytecount = data[1]; //4bytes are copied
                     bytesleft = bytecount;
                 break;
               case 2:

                  //lets count bytes recieved...
                  //we always receive/expect 8bytes here...
                  startbytereq+=len;
                  
                  //we recieve each canframe and add the 8 bytes to the buffer..
                  for(int i=0;i<2;i++){
                           page_buffer[pgwrite]=data[i];
                           pgwrite+=1; //lets increment
                  }  
                  
                  //ask for next chunk of page until page is complete ==> auto throttle
                  //we should add something for the last couple of bytes....
                  bytesleft -= len;

                  if ( (pgwrite!=SMALLEST_PAGE_WORDS) && (pgwrite!=PROGRAM_WORDS) && (bytesleft>0) ){                        
                        SendDataRequest(bytesleft);
                  }else if ((bytesleft>0)){
                     gbstate+=1;
                  }
                  else{ //this is a seperate step for debugging purposes (of the last partial page)
                     gbstate+=1;
                  }
                     
                 break; 
               case 3:
                     recvCrc=data[0];
                     gbstate+=1;
                 break;
               case 4:
                  //We just received the CRC wait here to signal main...
                  //
                 break;    
               default:
                 break; 
             }
               
         break;
      default:
         break;
    }


}

void waitGbstate(uint8_t waitForState){

    while (gbstate == waitForState)
         {
              wait(); 
         }

}

extern "C" int main(void)
{
   //This project is setup to boot the CPU at 64Mhz with 8Mhz external   
   clock_setup();
   
   rtc_setup();

   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,AFIO_MAPR_USART3_REMAP_NO_REMAP);//

   initialize_pins();

   nvic_setup();
   
   iwdg_set_period_ms(30000);
   iwdg_start();

   //this method is modified for our clockspeed...
   Can c(CAN1, Can::Baud500);
   
   //lets asign the can object to its pointers...
   canobj1 = &c; 

   //set the class
   c.SetReceiveCallback(CanCallback);

   // Set up CAN 1 callback and messages to listen for   
   c.RegisterUserMessage(masterCANID);//Master MSG

   numPages=0;
   actpage=0;
   bytecount = 0;
   startbytereq=0;

   wait();
   
   txmsg.id       = nodeCANID;
   txmsg.len      = 8;
   txmsg.data[0]  = '2';
   
   //ToDo: We could add the device name here...
   canSend(&txmsg);

   wait();

   if (magic == BOOTLOADER_MAGIC)
   {
      initbuffer();

      txmsg.data[0]  = 'S';
      canSend(&txmsg);
      
      //this assures that we receive numpages parameter from host//
      //don't forget our guard the watchdog...
      while (numPages == 0)
      {
         wait();
      }
      
      flash_unlock();

      while (actpage <= numPages)
      {
         recvCrc = 0;
         uint32_t timeOut = DELAY_200;

         crc_reset();
         
         //This channels incomeing data..
         gbstate=2;

         //we ask for the first 8bytes...
         SendDataRequest(8);   

         //wait for the dma page buffer tobe filled with data by the interface....
         //while (!dma_get_interrupt_flag(DMA1, USART_DMA_CHAN, DMA_TCIF))

         uint8_t recnr = 0;

         while (gbstate==2)
         {

            //if we recieved a piece we reset the counter and the watchdog.            
            if (recnr!=pgwrite){
               recnr=pgwrite;  
               timeOut = DELAY_200;
               iwdg_reset();
            }
            else
               timeOut--;

            //When the buffer is not full after about 200ms
            //Send a time-out to host. We rely on watchdog for upperstream time-out.
            if (0 == timeOut)
            {
               timeOut = DELAY_200;
               
               txmsg.data[0]='T';
               canSend(&txmsg);
            }

            
         }

         uint32_t crc = crc_calculate_block(&page_buffer[bufferOffset], receiveWords);

         SendCRCRequest();

         //wait to recieve the CRC value         
         waitGbstate(3);

         if (crc == recvCrc)
         {
            
            bufferOffset += receiveWords;
            
            /* Write to flash when we have sufficient amount of data or last page was received */
            if (bufferOffset == PROGRAM_WORDS || actpage == numPages)
            {
               write_flash(addr, page_buffer);
               addr += sizeof(page_buffer); 
               initbuffer();//lets zero RAM memory (for partial receive)
            }
            
            actpage+=1;

         }
         else
         {
            
            txmsg.data[0]  = 'E'; //ERROR!!!
            canSend(&txmsg);

         }
      }

      flash_lock();
   
      wait();

      //We are done lets tell the world this!!
      txmsg.data[0]  = 'D';
      canSend(&txmsg);

      wait();
      
   }  

   //ToDO: Maybe disable CAN on the otherhand the project already has CAN...
   //

   void (*app_main)(void) = (void (*)(void)) *(volatile uint32_t*)(APP_FLASH_START + 4);
   SCB_VTOR = APP_FLASH_START;
   app_main();

   return 0;
}

