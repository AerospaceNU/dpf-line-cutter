/*********************************************************************
line cutter boi:tm:
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

// pins for nichrome
const int PIN1 = A1;
const int PIN2 = A2;

// INCREASE THESE IF IT DOESN'T CUT FOR SOME REASON
const int POWER_LEVEL = 90;  // PWM uses this to turn on nichrome (should be between 0 and 255)
const int PWM_DURATION = 2000;  // length of pwm in milliseconds

void setup(void)
{
  // Turn off pins right away
  pinMode(PIN1, OUTPUT);
  analogWrite(PIN1, 0);
  pinMode(PIN2, OUTPUT);
  analogWrite(PIN2, 0);
  
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

  Serial.println("Bluetooth initialized.");
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

    // '!' indicates start of a command. anything else is ignored
    if (ch == (uint8_t) '!') {
      parse_command();
    }
  }
}


void parse_command() {
  uint8_t char_array[64];
  int current_num = 0;

  // read all text after '!' which started command
  while ( bleuart.available() && current_num<64 )
  {
    char_array[current_num] = (uint8_t) bleuart.read();
    current_num++;
  }
  
  String command = (char *) char_array;

  // keep track of whether each line cut has happened
  boolean status1 = false;
  boolean status2 = false;

  // if user typed '!cut', execute line-cutting
  if ( command.substring(0,3).equals("cut") ) {
    
    // enforce command structure: 'cut' with two 4-digit ints separated by spaces
    // it's... not completely robust. dw about it
    if ( command.substring(3,4).equals(" ") && command.substring(8,9).equals(" ") )
    {
      bleuart.write("Received cut command.\n");
    } else {
      bleuart.write("Please use format\n");
      bleuart.write("'cut xxxx xxxx'.\n");
      return;
    }

    // start cutting procedure
    status1 = cut_line( command.substring(4,8).toInt(), PIN1 );
    // if first cut was canceled, don't execute second cut
    if (status1) {
      status2 = cut_line( command.substring(9,13).toInt(), PIN2 );
    }

    // confirm status of each line
    bleuart.write("Line 1: ");
    if (status1) { bleuart.write("cut\n"); } else {bleuart.write("not cut\n"); }
    bleuart.write("Line 2: ");
    if (status2) { bleuart.write("cut\n"); } else {bleuart.write("not cut\n"); }
    return;
  }
  // anything other than 'cut' is invalid
  else {
    bleuart.write("Not a valid command.\n");
    return;
  }
}


boolean cut_line(int seconds, int pin) {
  boolean canceled = false;

  bleuart.write("Cutting line in ");
  bleuart.write( String(seconds).c_str() );
  bleuart.write(" seconds.\n");
  bleuart.write("'!' to cancel.\n");

  // countdown to line cut
  int time_elapsed = 0;
  while ( time_elapsed < seconds && !canceled ) {
    // check if user has sent a message
    while ( bleuart.available() && !canceled ) {
      uint8_t ch;
      ch = (uint8_t) bleuart.read();

      // if user sends '!', cancel cut
      if (ch == (uint8_t) '!') {
        canceled = true;
      }
    }
    delay(1000);
    time_elapsed++;
  }

  // proceed based on reason for loop ending
  if (canceled) {
    bleuart.write("Cut canceled.\n");
    return false;  // tell parse_command that line was not cut
  } else {
    execute_pwm(pin);
    return true;  // tell parse_command that line was cut
  }
}


void execute_pwm(int pin) {
  bleuart.write("Starting PWM on pin "); bleuart.write('0' + pin); bleuart.write('\n');
  Serial.println("Starting PWM ...");
  analogWrite(pin, POWER_LEVEL);
  delay(PWM_DURATION);
  analogWrite(pin, 0);
  bleuart.write("Done.\n");
  Serial.println("Done.");
  return;
}
