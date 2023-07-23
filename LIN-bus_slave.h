#include <Arduino.h>


#define BAUD_19200  19200
#define BAUD_9600   9600

#define WAIT_BREAK  1
#define WAIT_HIGH   2
#define GOT_DATA    3

#define CLASSIC_CS 0
#define ENHANCED_CS 1

typedef struct {
    uint8_t data[9];
    uint8_t slave_id;
    uint8_t len;
    uint8_t cs_type;
    uint8_t lock;
    uint8_t new_data;

} LIN_slave;


class lin_bus
{
  
  public:

  lin_bus(uint16_t baudrate); // Constructor 
  void event(void);
  int write(void);

  uint8_t get_slave_state(void);
  void set_slave_state(uint8_t state);
  uint8_t slave_read(uint8_t data[], uint8_t *lin_length);
  void update_slave_data(uint8_t data[],uint8_t ident, uint8_t len, uint8_t cs_type);
  static void rxISR(void);
  void slave_init(void);

  
  private:
  uint8_t addrParity(uint8_t addr);
  uint8_t dataChecksum(const uint8_t* message, uint8_t nBytes,uint16_t sum);
  uint8_t tx_pin;
  uint16_t bit_len;
  
  
};
