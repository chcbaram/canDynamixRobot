#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "mblock.h"



#define MBLOCK_SERIAL     Serial



void (*mblock_read_sensor_callback)(mblcok_packet_t *p_packet, uint8_t device) = NULL;
void (*mblock_run_module_callback)(mblcok_packet_t *p_packet, uint8_t device) = NULL;







mblcok_packet_t mblock_packet;



static bool mblockReceivePacket(uint8_t rx_data);



//ff 55 len idx action device port  slot  data a
//0  1  2   3   4      5      6     7     8



void mblockBegin(uint32_t baud)
{
  MBLOCK_SERIAL.begin(baud);

  mblock_packet.state = 0;
}

bool mblockUpdate(void)
{
  bool ret = false;

  if (MBLOCK_SERIAL.available() > 0)
  {
    ret = mblockReceivePacket(MBLOCK_SERIAL.read());
  }


  if (ret == true)
  {
    switch(mblock_packet.action)
    {
      case GET:
        MBLOCK_SERIAL.write(0xFF);
        MBLOCK_SERIAL.write(0x55);        
        MBLOCK_SERIAL.write(mblock_packet.idx);

        if (mblock_read_sensor_callback != NULL)
        {
          (*mblock_read_sensor_callback)(&mblock_packet, mblock_packet.device);
        }
        MBLOCK_SERIAL.println();
        break;

      case RUN:
        if (mblock_run_module_callback != NULL)
        {
          (*mblock_run_module_callback)(&mblock_packet, mblock_packet.device);
        }
        MBLOCK_SERIAL.write(0xFF);
        MBLOCK_SERIAL.write(0x55);  
        MBLOCK_SERIAL.println();
        break;

      case RESET:
        break;
      
      case START:
        MBLOCK_SERIAL.write(0xFF);
        MBLOCK_SERIAL.write(0x55);  
        MBLOCK_SERIAL.println();
        break;
    }    
  }

  return ret;
}

bool mblockReceivePacket(uint8_t rx_data)
{
  bool ret = false;


  switch(mblock_packet.state)
  {
    case 0:
      if (rx_data == 0xFF)
      {
        mblock_packet.state = 1;
      }
      break;

    case 1:
      if (rx_data == 0x55)
      {
        mblock_packet.state = 2;
      }
      else
      {
        mblock_packet.state = 0;
      }
      break;

    case 2:
      mblock_packet.length = rx_data;
      mblock_packet.rx_index = 0;
      mblock_packet.state = 3;
      break;

    case 3:
      mblock_packet.buffer[3 + mblock_packet.rx_index] = rx_data;    
      mblock_packet.rx_index++;

      if (mblock_packet.rx_index >= mblock_packet.length)
      {
        mblock_packet.idx    = mblock_packet.buffer[3];
        mblock_packet.action = mblock_packet.buffer[4];
        mblock_packet.device = mblock_packet.buffer[5];
        mblock_packet.port   = mblock_packet.buffer[6];
        mblock_packet.slot   = mblock_packet.buffer[7];

        ret = true;
        mblock_packet.state = 0;
      }
      break;
  }


  return ret;
}