/*
Teensy 4.0 LIN-bus slave device emulation
skpang.co.uk 07/23

For use with:
https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-can-fd-and-lin-bus-breakout-board-include-teensy-4-0


*/

#include "LIN-bus_slave.h"
int lin_cs = 32;

int lin_fault = 28;
uint8_t lin_length;

uint16_t lin_count = 0;
uint8_t lin_slave_data[8]; 
uint8_t lin_slave_data_size = 0;
uint8_t lin_slave_addr = 0;

uint16_t task1 = 0;
uint16_t task2 = 0;

lin_bus lin(BAUD_19200);
IntervalTimer timer;

void sys_tick(void)
{
  task1++;
  task2++;

}

void setup() {
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(lin_fault,INPUT);
  pinMode(lin_cs, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(lin_cs, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  Serial.println("Teensy 4.0 LIN-bus slave device demo skpang.co.uk 07/23");

  timer.begin(sys_tick,1000);
  lin.slave_init();

}

void loop() {

  lin.event();

  if(task2 > 500)
  {
      task2 = 0;

      // Update slave device data here
      lin_slave_data_size = 8;
      lin_slave_addr = 0x49;       // It will add parity for you automatically
      lin_slave_data[0] = lin_count;
      lin_slave_data[1] =  (0xff00 & lin_count) >> 8;
      lin_slave_data[2] = 0x34;
      lin_slave_data[3] = 0x56;
      lin_slave_data[4] = 0x78;
      lin_slave_data[5] = 0x9a;
      lin_slave_data[6] = 0xbc;
      lin_slave_data[7] = 0xde;
      lin.update_slave_data(lin_slave_data, lin_slave_addr, lin_slave_data_size, ENHANCED_CS);

      lin_count++;

  }

  if(task1 > 500)
  {
    task1 = 0;
    digitalToggle(LED_BUILTIN);
  }
  
}
