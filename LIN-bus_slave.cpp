/*
Teensy 4.0 LIN-bus slave device emulation
skpang.co.uk 07/23

For use with:
https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-can-fd-and-lin-bus-breakout-board-include-teensy-4-0


*/

#include "LIN-bus_slave.h"

uint8_t rx_state;
uint32_t kbps;
uint8_t tx_pin = 14;
uint8_t rx_pin = 15;
uint8_t lin_data[8];
uint8_t slave_data_len = 0;

int led1 = 23;
elapsedMillis timeout;
elapsedMicros break_length;

LIN_slave lin_slave;
LIN_slave lin_slave_old;

lin_bus::lin_bus(uint16_t baudrate)
{
  kbps = baudrate;
}

void lin_bus::event(void)
{
  uint8_t data[10]; 
  uint8_t lin_length,i;

  if((lin_slave.new_data == 1) && (lin_slave.lock == 0))
    {
        lin_slave_old.cs_type = lin_slave.cs_type;
        lin_slave_old.len = lin_slave.len;
        lin_slave_old.slave_id = lin_slave.slave_id;
        
        for(i = 0;i<lin_slave.len;i++)
            lin_slave_old.data[i] = lin_slave.data[i];
        
        lin_slave.new_data = 0;    
    }

  if(get_slave_state() == GOT_DATA)
   {
      slave_read(data,&lin_length);
    
      // Check ID is for this slave or not
      if(data[1] == lin_slave_old.slave_id)  // The second byte from the the serial port is the LIN ID 
      {
          write();  // Write slave data out
          Serial.print("Received LIN ID: ");
          Serial.print(data[1],HEX);
          Serial.println(" ");
      }  
    }         
}

void lin_bus::update_slave_data(uint8_t data[],uint8_t ident, uint8_t len, uint8_t cs_type)
{
    uint8_t i;
    uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);
    lin_slave.lock = 1;     // Set lock to prevent update
    
    lin_slave.len = len;
    lin_slave.cs_type = cs_type;
    lin_slave.slave_id = addrbyte;

    for (i = 0; i < len; i++) {
      lin_slave.data[i] = data[i];   //lin_buff[i] = temp;
    }

    lin_slave.lock = 0;    // Set unlock
    lin_slave.new_data = 1;
}

int lin_bus::write(void)
{
  uint8_t checksum;

  if(lin_slave_old.cs_type == ENHANCED_CS)
  {
      checksum = dataChecksum(lin_slave_old.data, lin_slave_old.len,lin_slave_old.slave_id);
  }else checksum = dataChecksum(lin_slave_old.data, lin_slave_old.len,0);

  Serial3.begin(kbps);      // Configure baudrate

  for (uint8_t i = 0; i < lin_slave_old.len; i++) Serial3.write(lin_slave_old.data[i]); // write data to serial
  Serial3.write(checksum);

  Serial3.end();
  digitalWriteFast(led1, LOW);
  return 1;
}


/* ISR for measuring break
 *  
 *  Line break for 19200 is 675us
 *                 9600 is 1354us
 * 
 */
void lin_bus::rxISR(void)
{
  
  if (rx_state == WAIT_BREAK)
  {
    if (digitalReadFast(rx_pin) == 0)
    {
    //  digitalWrite(11, LOW); 
      break_length = 0;
      rx_state = WAIT_HIGH;
     
    }
    digitalWriteFast(led1, HIGH);
  }
  
  if (rx_state == WAIT_HIGH)
  {
    if (digitalReadFast(rx_pin) == 1)
    {
      if (kbps == BAUD_19200)
      { 
          if((break_length > 600) && (break_length < 700))
          {
              Serial3.begin(BAUD_19200);        // Configure serial baudrate 
          } else
          {
             //Serial.println("Wrong 19200 break ");
             rx_state = WAIT_BREAK;
          }
      }else 
      {

        if((break_length > 1250) && (break_length < 1400))
          {
              Serial3.begin(BAUD_9600);;        // Configure serial baudrate 
          }else
          {
            // Serial.println("Wrong 9600 break ");
             rx_state = WAIT_BREAK;
          }
      
      }
      Serial3.clear();
      rx_state = GOT_DATA;

    }
  }
}

void lin_bus::slave_init(void)
{
  lin_slave_old.lock = 0;
  lin_slave_old.new_data = 0;

  pinMode(rx_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rx_pin), rxISR, CHANGE);

  rx_state = WAIT_BREAK;

}

uint8_t lin_bus::get_slave_state(void)
{
  return rx_state;
}


void lin_bus::set_slave_state(uint8_t state)
{
  rx_state = state;
  pinMode(rx_pin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(rx_pin),rxISR,CHANGE);
}


uint8_t lin_bus::slave_read(uint8_t data[], uint8_t *lin_length)
{
  uint8_t i = 0;
  uint8_t rx;
  uint8_t rtn = 0;
  

  timeout = 0;
  while (i < 11)  // 11 characters to receive. Sync(0x55) + ID + Data(8 bytes) + CS = 11
  {
    if (Serial3.available())
    {
      rx = Serial3.read();
      data[i] = rx;
      i++;
      *lin_length = i;
      rtn = 1;
    }

    if (kbps == BAUD_19200)
    { 
      if (timeout > 8) // 8ms timeout for 19200 baud
      {
        //Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
    }else
    {
      if (timeout > 14) // 14ms timeout for 9600 baud
      {
        //Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
      
      
    }
  }
  
  set_slave_state(WAIT_BREAK);
  return rtn;
}

#define BIT(data,shift) ((addr&(1<<shift))>>shift)
uint8_t lin_bus::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr, 0) ^ BIT(addr, 1) ^ BIT(addr, 2) ^ BIT(addr, 4);
  uint8_t p1 = ~(BIT(addr, 1) ^ BIT(addr, 3) ^ BIT(addr, 4) ^ BIT(addr, 5));
  return (p0 | (p1 << 1)) << 6;

}

uint8_t lin_bus::dataChecksum(const uint8_t* message, uint8_t nBytes, uint16_t sum)
{

  while (nBytes-- > 0) sum += *(message++);
  // Add the carry
  while (sum >> 8) // In case adding the carry causes another carry
    sum = (sum & 255) + (sum >> 8);
  return (~sum);

}
