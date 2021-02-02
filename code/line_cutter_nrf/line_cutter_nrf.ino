/*********************************************************************
To Do:
- comments lol
- feedback messages over uart not serial
- enforce structure of cut command 
- test w/ nichrome
*********************************************************************/

#include <bluefruit.h>

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer[];

// pins
const int PIN1 = A1;
const int PIN2 = A2;

void setup(void)
{
  // Turn off pins right away
  pinMode(PIN1, OUTPUT);
  digitalWrite(PIN1, LOW);
  pinMode(PIN2, OUTPUT);
  digitalWrite(PIN2, LOW);
  
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println("Initialized!");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();

    if (ch == (uint8_t) '!') {
      parse_command();
    }
  }
}

void parse_command() {
  uint8_t char_array[64];
  int current_num = 0;
 
  while ( bleuart.available() && current_num<64 )
  {
    char_array[current_num] = (uint8_t) bleuart.read();
    current_num++;
  }
  
  String command = (char *) char_array;

  boolean status1 = false;
  boolean status2 = false;
  
  if ( command.substring(0,3).equals("cut") ) {
    bleuart.write("Received cut command.\n");
    status1 = cut_line( command.substring(4,8).toInt(), PIN1 );
    if (status1) {
      status2 = cut_line( command.substring(9,13).toInt(), PIN2 );
    }
    bleuart.write("Line 1: ");
    if (status1) { bleuart.write("cut\n"); } else {bleuart.write("not cut\n"); }
    bleuart.write("Line 2: ");
    if (status2) { bleuart.write("cut\n"); } else {bleuart.write("not cut\n"); }
    return;
  }
  else {
    bleuart.write("Not a valid command.\n");
    return;
  }
}

boolean cut_line(int seconds, int pin) {
  boolean canceled = false;

  bleuart.write("Line will be cut in ");
  bleuart.write( String(seconds).c_str() );
  bleuart.write(" seconds.\n");
  bleuart.write("'!' to cancel.\n");
  int time_elapsed = 0;
  while ( time_elapsed < seconds && !canceled ) {
    while ( bleuart.available() && !canceled ) {
      uint8_t ch;
      ch = (uint8_t) bleuart.read();

      if (ch == (uint8_t) '!') {
        canceled = true;
      }
    }
    delay(1000);
    time_elapsed++;
  }
  if (canceled) {
    bleuart.write("Cut canceled.\n");
    return false;
  } else {
    execute_pwm(pin);  // on first line
    return true;
  }
}

void execute_pwm(int pin) {
  bleuart.write("Starting PWM on pin "); bleuart.write('0' + pin); bleuart.write('\n');
  Serial.println("Starting PWM ...");
  analogWrite(pin, 60);
  delay(2000);
  analogWrite(pin, LOW);
  bleuart.write("Done.\n");
  Serial.println("Done.");
  return;
}
