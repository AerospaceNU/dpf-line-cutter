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
const int VOLTAGE_DIVIDER = A0;
const int PIN1 = A1;
const int PIN2 = A2;

// INCREASE THESE IF IT DOESN'T CUT FOR SOME REASON
const double POWER_LEVEL1 = 0.3;  // PWM uses this voltages to turn on nichrome
const double POWER_LEVEL2 = POWER_LEVEL1;
const int PWM_DURATION = 2000;  // length of pwm in milliseconds

void setup(void)
{
  // Turn off pins right away
  // Except that nrfAnalogWrite doesn't actually turn pins off
  // kekw
  pinMode(PIN1, OUTPUT);
  nrfAnalogWrite(PIN1, 0);
  pinMode(PIN2, OUTPUT);
  nrfAnalogWrite(PIN2, 0);
  
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Condor");

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
    status1 = cut_line( command.substring(4,8).toInt(), PIN1, POWER_LEVEL1 );
    // if first cut was canceled, don't execute second cut
    if (status1) {
      status2 = cut_line( command.substring(9,13).toInt(), PIN2, POWER_LEVEL2 );
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


boolean cut_line(int seconds, int pin, double targetVoltage) {
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
    execute_pwm(pin, targetVoltage);
    return true;  // tell parse_command that line was cut
  }
}


void execute_pwm(int pin, double targetVoltage) {
  bleuart.write("Starting PWM on pin "); bleuart.write('0' + pin); bleuart.write('\n');
  Serial.println("Starting PWM ...");
  nrfAnalogWrite(pin, pwmLevel(targetVoltage));
  delay(PWM_DURATION);
  nrfAnalogWrite(pin, 0);
  bleuart.write("Done.\n");
  Serial.println("Done.");
  return;
}

// calculate number to be used for nrfAnalogWrite() of nichrome pin
// double -> int
int pwmLevel(double targetVoltage) {
  // get reading from voltage divider and multiply by 2
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  // to get current vbat  
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // find proportion of vbat needed to apply target voltage, then make it out of 255 for nrfAnalogWrite

  Serial.print("Target "); Serial.print(targetVoltage); Serial.print(" vbat "); Serial.print(vbat); 
  
  auto ret = (targetVoltage / vbat) * 255;
  Serial.print(" pwm "); Serial.println(ret);
  return ret;
}


/**
 * Generate PWM without pre-configured. this function will
 * configure pin to available HardwarePWM and start it if not started
 * 
 * This version of the method seems to not try to burn lines when initilizing analog pins, which is always a good thing :)
 *
 * @param pin
 * @param value
 */
void nrfAnalogWrite( uint32_t pin, uint32_t value )
{
  // first, handle the case where the pin is already in use by nrfAnalogWrite()
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if (HwPWMx[i]->isOwner(0x676f6c41))
    {
      int const ch = HwPWMx[i]->pin2channel(pin);
      if (ch >= 0)
      {
        HwPWMx[i]->writeChannel(ch, value);
        return;
      }
    }
  }

  // Next, handle the case where can add the pin to a PWM instance already owned by nrfAnalogWrite()
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if ( HwPWMx[i]->isOwner(0x676f6c41) && HwPWMx[i]->addPin(pin) )
    {
      // successfully added the pin, so write the value also
      HwPWMx[i]->writePin(pin, value);
      LOG_LV2("Analog", "Added pin %" PRIu32 " to already-owned PWM %d", pin, i);
      return;
    }
  }

  // Attempt to acquire a new HwPWMx instance ... but only where
  // 1. it's not one already used for analog, and
  // 2. it currently has no pins in use.
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if (HwPWMx[i]->takeOwnership(0x676f6c41))
    {
      // apply the cached analog resolution to newly owned instances
      HwPWMx[i]->setResolution(10);
      HwPWMx[i]->stop();
      HwPWMx[i]->addPin(pin);
      HwPWMx[i]->writePin(pin, value);
      HwPWMx[i]->begin();
      LOG_LV2("Analog", "took ownership of, and added pin %" PRIu32 " to, PWM %d", pin, i);
      return;
    }
  }

  LOG_LV1("Analog", "Unable to find a free PWM peripheral");
  return;
}
