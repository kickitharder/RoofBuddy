#include <Arduino.h>
#define VERSION "RoofBuddy V0.231230 by keithrickard@hotmail.com"
  
/* Work commenced 14 Jan 2022 - For the Arduino Uno

            +-----------------------+
            |                       |
            |                       o A5_SCL
            |                       o A4_SDA
            o                       o AREF
            o                       o GND
            o                       o 13  BUZZER
        RST o                       o 12  DIR 
        3V3 o                       o 11~ PWM
         5V o                       o 10~ LED
        GND o                       o 9~  TB_CXN
        GND o                       o 8   TB_TXD
+12V    VIN o                       o 7   TB_RXD
            |                       o 6~  LX_CXN
BATTERY  A0 o                       o 5~  LX_TXD
BTN_MOVE A1 o                       o 4   LX_RXD
BTN_STOP A2 o                       o 3~  SW_CLOSED
RG9      A3 o                       o 2   SW_OPEN
RG9_VCC  A4 o         ooo1          o 1   TX
RG9_VCC  A5 o         ooo           o 0   RX
            +-----------------------+

      ICSP
      ooo1    RST D13 D12  
      ooo     GND D11 5V
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>

#define SW_CLOSED   2             // Microswitch to indicate if roof is fully closed
#define SW_OPEN     3             // Microswitch to indicate if roof is fully open
#define LX_RXD      4             // LX200 Bluetooth module comms lines
#define LX_TXD      5             
#define LX_CXN      6             // Indicates connection to LX200
#define TB_RXD      7             // TopBox Bluetooth module comms lines
#define TB_TXD      8
#define TB_CXN      9             // Indicates connection to TopBox
#define LED         10            // Indicates RoofBuddy controller status
#define PWM         11            // Motor PWM
#define DIR         12            // Motor direction
#define BUZZER      13            // Buzzer

#define BATTERY     A0            // Battery charge sensor
#define BTN_OPEN    A1            // Control button to start opening the roof
#define BTN_CLOSE   A2            // Control button to start closing the roof
#define RG9         A3            // Rain sensor's output line.  1 = dry, 0 = wet
#define RG9_VCC1    A4            // Power for the RG-9 Rain Sensor
#define RG9_VCC2    A5            // 

#define PWM_MAX     192           // Maximum motor speed
#define PWM_DEC     1             // PWM steps when ramping down
#define PWM_INC     4             // PWM steps when ramping up
#define PWM_DELAY   1             // Delay between PWM increments for ramping up
#define SENSE_ADJ   60.9          // Adjust to get correct battery voltage reading
#define PWM_HALT    2             // Delay for rapid halt of the moving roof
#define PRESSED     1             // Value if control button is pressed (1 for normally closed, 0 for normally open)
#define BATT_GOOD   12.3          // If BATT_LVL is less than this value then the battery needs charging.
#define BATT_BAD    6             // Do not close the roof if battery voltage is below this
#define MOVE_PERIOD 50            // Maximum time (secs) to carry out move operation
#define DIR_OPEN    1             // Parameter for the direction to open the roof
#define DIR_CLOSE   0             // Parameter for the direction to close the roof.
#define PARK_PERIOD 1000
#define OPENED        'O'         // Roof statuses
#define CLOSED        'C'
#define OPENING       'o'
#define CLOSING       'c'
#define STOP_OPEN     'h'
#define STOP_CLOSE    'H'
#define PARK_OPEN     'p'
#define PARK_CLOSE    'P'
#define BATT_OPEN     'b'
#define BATT_CLOSE    'B'
#define TIME_OPEN     't'
#define TIME_CLOSE    'T'
#define RAIN_OPEN     'r'
#define RAIN_CLOSE    'R'
#define SAFE_OPEN     's'
#define SAFE_CLOSE    'S'
#define LXCXN_OPEN    'l'
#define LXCXN_CLOSE   'L'
#define NOPARK_OPEN   'u'
#define NOPARK_CLOSE  'U'
#define PARK_HA     0.0           // Default telescope park position
#define PARK_DEC    -35

SoftwareSerial LX(LX_TXD, LX_RXD);      // Bluetooth serial for FocusBuddy
SoftwareSerial TB(TB_TXD, TB_RXD);      // Bluetooth serial for LX200

volatile int pwm;                       // 0 = stopped, >0 = moving
volatile bool isr;                      // 1 = interrupt triggered
int pwmFlag;

char cmd = 0;
char lastCmd = 0;
byte scopeSafe = 0;                     // Assume scope not in safe position
byte dir = DIR_CLOSE;                   // 0 = close, 1 = open
byte serial = 0;                        // 0 = Serial, 1 = TB
byte stillParking = 0;
char parkMode = '\0';
int maxSpeed;
byte tbBtn, flashType, flashPos, timedOut, itsRaining, rainSensorPwr;
unsigned long rampTimer, buttonTimer, flashTimer, moveTimer, parkingTimer, buzzerTimer, safetyTimer, rainTimer;

bool test = 0;
byte sw_open = 0;
byte sw_closed = 0;
byte testSw_open = 1;
byte testSw_closed = 0;
byte safe_T = 1;
byte park_T = 1;
byte lx200_T = 1;
int batt_T = 780;
int rain_T = 0;                       // 0 = Dry, 1 = Rain

struct ee {
  float ha = 0.0;
  int dec = -35;
  byte movePeriod = MOVE_PERIOD;
  byte maxSpeed = PWM_MAX;
  byte park = 1;                      // Ensure telescope should park
  byte isParked = 1;                  // Ensure is parked
  byte safe = 1;                      // Ensure scope is safe
  byte rain = 1;                      // Enusre rain sensor is monitored
  byte battery = 1;                   // Ensure there is enough power
} ee;
//==================================================================================================================================================
void setup() {
  pinMode(DIR, OUTPUT);               // Motor controller board DIR pin (direction)
  pinMode(PWM, OUTPUT);               // Motor controller board PWM pin (speed)
  pinMode(BUZZER, OUTPUT);
  pinMode(SW_OPEN, INPUT_PULLUP);     // If 1 = Closed, 0 = Open
  pinMode(SW_CLOSED, INPUT_PULLUP);   // If 1 = Closed, 0 = Open
  pinMode(BTN_OPEN, INPUT_PULLUP);    // If 1 = Pressed, 0 = Released
  pinMode(BTN_CLOSE, INPUT_PULLUP);   // If 1 = Pressed, 0 = Released
  pinMode(RG9, INPUT_PULLUP);         // Power up the RG-9 rain sensor
  pinMode(RG9_VCC1, OUTPUT);
  pinMode(RG9_VCC2, OUTPUT);

  Serial.begin(9600);
  Serial.println(VERSION);

  lastCmd = tbBtn = flashType = flashPos = rainSensorPwr = timedOut = itsRaining = pwmFlag = pwm = moveTimer = rainTimer = safetyTimer = 0;
  motorControl();
  rainSensor(2);                      // Switch on rain sensor
  TB.begin(9600);
  LX.begin(9600);
  TB.setTimeout(2000);
  LX.setTimeout(1000);

  digitalWrite(BUZZER, HIGH);
  if (button(BTN_OPEN)){              // Enter button test mode
    buzz(1000);
    hardwareTest();
  }
  if (button(BTN_CLOSE)){             // Enter override mode
    buzz(1000);
    override();
  }
  readEE();
  TB.listen();
  buzz(-100);
  dir = (roofClosed() ? DIR_CLOSE : DIR_OPEN);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  serviceLED();
  actIfRaining();
  digitalWrite(BUZZER, (millis() < buzzerTimer));                             // Turn off the buzzer if its timer has expired
  TB.listen();
  moveRoof();                                                                 // Service the moving of the roof
  if (TB.available()) commands(1);                                            // See if the TopBox is talking to us
  if (Serial.available()) commands(0);                                        // See if the PC is talking to us.
  if (stillParking) parkingCheck();                                           // See if LX200 is parking
  if (!roofClosed()) rainSensor(1);                                           // If roof is not closed then turn on the rain sensor
  if (pwm) if (!tbBtn) if (safetyTimer) if (millis() > safetyTimer) halt(10); // If no safety info received then stop moving
  if (!test) {
    testSw_open = digitalRead(SW_OPEN);
    testSw_closed = digitalRead(SW_CLOSED);
  }
}
//==================================================================================================================================================
// COMMANDS
//==================================================================================================================================================
void commands(byte s) {
  serial = s;
  cmd = serial ? TB.read() : Serial.read();
  if (cmd != '$') {
    Serial.print("Received from "); Serial.print(s ? "TopBox: " : "PC: "); Serial.println(cmd);
  }
  switch (cmd) {
    case '^': return;                     break;    // Soak up prefix character if if exists
    case '\n': return;                    break;    // Soak up lineeed character
    case '\r': return;                    break;    // Soak up carriage return character
    case '$': return;                     break;    // Soak up '$' terminator character
    case 'e': rainSensor(1);              break;    // Returns 'e' meaning RoofBuddy is responding. Also, turn on the rain sensor
    case 'O': openRoof();                 break;    // Open roof completely
    case 'o': topBoxOpen();               break;    // TopBox open button has been pressed to open roof
    case 'C': closeRoof();                break;    // Close roof completely
    case 'c': topBoxClose();              break;    // TopBox open button has been pressed to close roof
    case 's': status();                   break;    // 'C' = Closed, 'c' = closing, 'O' = Opened, 'o' = Opening, 'H' = Stopped, etc
    case 'w': swStatus();                 break;    // 'M' = both switches open, 'O' open switch closed, 'C' close switch closed
    case 'b': getBattery();               break;    // Returns battery voltage. >= 12.5V = OK, < 12.5V = Charge battery
    case 'R': rainSensor(2);              break;    // Restart Rain Sensor
    case 'r': getRainSensor();            break;    // Returns: 1 = wet, 0 = dry
    case 'I': itsRaining = 1;             break;
    case 'i': itsRaining = 0;             break;
    case 'M': setMoveTimer();             break;    // Set max time to move roof: 0 to 255 seconds
    case 'm': getMoveTimer();             break;    // Get roof mover timer value
    case 'X': setMaxSpeed();              break;    // Set maximum motor speed
    case 'x': getMaxSpeed();              break;    // Get maximum motor speed
    case 'g': getMovePercent();           break;    // Get estimated time to go to move roof as a percentage
    case 'U': setParameters();            break;    // 'P'/'p' Park, 'I'/'i' is parked, 'A'/'a' accelerometer, 'B'/'b' battery
    case 'u': getParameters();            break;    // 'P' Park, 'I' is parked, 'A' accelerometer, 'B' battery
    case 'H': halt(2);                    break;    // Stop motor immediately
    case 'h': slowHalt(1);                break;    // Bring motor to a slow stop
    case 'D': setDEC();                   break;    // Store and use this declination for parking to EEPROM
    case 'd': getDEC();                   break;    // Get stored declination for parking
    case 'A': setHA();                    break;    // Store and use this hour angle for parking to EEPROM
    case 'a': getHA();                    break;    // Get stored hour angle for parking
    case 'L': cmd = lastCmd;              break;    // Get last command response
    case 'l': cmd = lx200();              break;    // Is telescope connected and responding? '1' = yes, '0' = no
    case '_': startParking();             break;    // Instruct LX200 to slew to home position
    case 'p': cmd = isParked();           break;    // Is LX200 parked? '1' = yes, '0' = no
    case 'Q': stopParking();              break;    // Stop parking the telescope
    case 'V': version();                  break;    // Send version
    case 'v': variables();                break;    // Send this sketch's variables
    case 'T': scopeSafeStatus(1); return; break;    // Telescope is safely positioned to move roof
    case 't': scopeSafeStatus(0); return; break;    // Telescope is not safely positioned to move roof
    case '~': hardware();                 break;    // Show Arduino's pin statuses
    case ':': lx200cmd();                 break;    // Send a command directly to the LX200 and send back any response
    case 'Z': testCmds();                 break;    // Test mode commands
    case 'F': formatEE();                 break;    // Re-format EEPROM
    case 'S': Serial.println(scopeSafe);  break;    // Return what RB thinks is the scope safety
    case '/': asm volatile ("jmp 0");     break;    // Restart the Arduino
    default: cmd = '%';                   break;    // Say what?
  }
  if (cmd) {
    lastCmd = cmd;
    if (serial) {
      TB.listen();
      TB.print(cmd);
      Serial.print(F("Sent to TopBox: "));
    }
    Serial.print(cmd);
  }
  TB.print('$');
  Serial.println('$');
  TB.flush();
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void testCmds() {         // Z
  Serial.print(F("testCmds() => Received: "));
  char buf[] = "\0\0\0";
  serial ? TB.readBytesUntil('$', buf, 1) : Serial.readBytesUntil('$', buf, 1);
  cmd = buf[0];
  Serial.println(cmd);

  switch (cmd) {
    //case '>':   roofO();            break;  // 'roof open'
    //case '<':   roofC();            break;  // 'roof closed'
    case '>':   testSw_closed = 1;  break;  // 'roof not open'
    case '<':   testSw_open= 1;     break;  // 'roof not closed'
    case 'O':   roofO();            break;  // 'roof open'
    case 'C':   roofC();            break;  // 'roof closed'
    case 'H':   roofH();            break;  // 'roof partly open'
    case '-':   roofH();            break;  // 'roof partly open'
    case 'b':   battT(0);           break;  // 'battery bad'
    case 'B':   battT(1);           break;  // 'battery good'
    case 'r':   rainT(0);           break;  // 'not raining'
    case 'R':   rainT(1);           break;  // 'raining'
    case 'P':   parkT(1);           break;  // 'scope' not parked
    case 'p':   parkT(0);           break;  // 'scope' parked
    case 'l':   lx200T(0);          break;  // 'LX200 not repsonding'
    case 'L':   lx200T(1);          break;  // 'LX200 responding'
    case 's':   safeT(0);           break;  // 'scope' not safe
    case 'S':   safeT(1);           break;  // 'scope' safe
    case 'v':   testStatus();       break;  // Show test mode status
    case '?':   testStatus();       break;  // Show test mode status
    case 'T':                               // Flip test mode status
      test = !test;
      sw_closed = sw_open = 0;
      Serial.print(F("=> ")); Serial.println(test ? F("TEST MODE") : F("LIVE MODE"));
      break;
    default:  break;
  }
  return;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getTerm() {
  if (!cmd) return;
  unsigned long findTimer = millis() + 100;
  while (millis() < findTimer) {
    switch (serial ? TB.read() : Serial.read()) {
      case '$':   return;                   break;
      case '\n':  return;                   break;
      case '\r':  return;                   break;
      case -1:                              break;
      default: findTimer = millis() + 100;  break;
    }
  }
}
//==================================================================================================================================================
// ROOF STUFF
//==================================================================================================================================================
void openRoof() {         // ^O
  rainSensor(3);                                            // Turn on the roof sensor
  if (cmd) getTerm();
  Serial.print(F("openRoof() "));                           // Try and get the roof to start opening

  parkMode = 0;
  if (roofOpen()){                                          // Quit if roof is already opened
    Serial.println(F("=> ROOF ALREADY OPENED"));
    dir = DIR_OPEN;
    if (!cmd) cmd = OPENED;
    return;
  }
  if (startedRaining()){
    Serial.println(F("=> RAIN - OPENING ABORT"));
    halt(1);
    cmd = RAIN_OPEN;
    return;
  }
  if (!batteryCheck()) {
    Serial.println(F("=> NO POWER"));
    dir = DIR_OPEN;
    if (!cmd) cmd = BATT_OPEN;
    return;
  }
  if (dir == DIR_OPEN && pwm) {
    Serial.println(F("=> ROOF ALREADY OPENING"));
    if (cmd) cmd = OPENING;
    return;
  }

  Serial.println(F("=> OPENING ROOF"));
  if (dir == DIR_CLOSE && pwm) slowHalt(2);                 // If the roof is closing then stop it before opening it.
  dir = DIR_OPEN;                                           // Direction to open roof
  timedOut = 0;                                             // Reset timeOut flag
  if (!cmd && !itsRaining) {
    TB.print(F("o$"));                                      // Let TopBox know command is being acted on
    Serial.println(F("o$"));
  }

  if (tbBtn || scopeSafeCheck()) {                          // If scope safe or TopBox open button has been pressed then close
    if (cmd) cmd = OPENING;
    sw_open = sw_closed = pwmFlag = 0;                      // Reset test limit switches
    pwm = 1;                                                // Start moving the roof
    buzz(-500);
    rampTimer = millis() + PWM_DELAY;                       // Set ramp-up timer
    moveTimer = millis() + ee.movePeriod * 1000L;           // Set maximum roof move timer
  }
  if (!tbBtn && !stillParking) {
    parkMode = PARK_OPEN;
    switch (isParked()) {                                   // '0' = Not parked, '1' = parked, '2' = unknown
      case '0': cmd = PARK_OPEN;  startParking(); break;    // Telecope not parked - park it
      case '1': cmd = OPENING;    parkMode = 0;   break;    // Telescope parked
      case '2': cmd = LXCXN_OPEN;                 break;    // Telescope not responding
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void closeRoof() {        // ^C
  if (cmd) getTerm();
  Serial.print(F("closeRoof() "));                          // Try and get the roof to start closing

  parkMode = 0;
  if (roofClosed()) {                                       // Quit if roof is already closed
    Serial.println(F("=> ROOF ALREADY CLOSED"));
    dir = DIR_CLOSE;
    if (!cmd) cmd = CLOSED;
    return;
  }
  if (!batteryCheck()) {
    Serial.println(F("=> NO POWER"));
    dir = DIR_CLOSE;
    if (!cmd) cmd = BATT_CLOSE;
    return;
  }
  if (dir == DIR_CLOSE && pwm) {
    Serial.println(F("=> ROOF ALREADY CLOSING"));
    if (cmd) cmd = CLOSING;
    return;
  }

  Serial.println(F("=> CLOSING ROOF"));
  if (dir == DIR_OPEN && pwm) slowHalt(3);                  // If the roof is opening then stop it before closing it.
  dir = DIR_CLOSE;                                          // Direction to close roof
  timedOut = 0;                                             // Reset timeOut flag
  if (!cmd && !itsRaining) {
    TB.print(F("c$"));                                      // Let TopBox know command is being acted on
    Serial.println(F("c$"));
  }

  if (tbBtn || scopeSafeCheck()) {                          // If scope safe or TopBox close button has been pressed then close
    if (cmd) cmd = CLOSING;
    sw_open = sw_closed = pwmFlag = 0;                      // Reset test limit switches
    pwm = 1;                                                // Start moving the roof
    buzz(-500);
    rampTimer = millis() + PWM_DELAY;                       // Set ramp-up timer
    moveTimer = millis() + ee.movePeriod * 1000L;           // Set maximum roof move timer
  }
  if (!tbBtn && !stillParking) {
    parkMode = PARK_CLOSE;
    switch (isParked()) {                                   // '0' = Not parked, '1' = parked, '2' = unknown
      case '0': cmd = PARK_CLOSE;  startParking();  break;  // Telescope not parked - park it
      case '1': cmd = CLOSING;     parkMode = 0;    break;  // Telescope parked
      case '2': cmd = LXCXN_OPEN;                   break;  // Telescope not responding
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void topBoxOpen() {       // ^o
  Serial.println(F("topBoxOpen()"));                        // The OPEN button on the TopBox is being pressed
  tbBtn = 1;                                                // TopBox button invoked this action
  buzz(-100);
  openRoof();                                               // Try and get the roof to open
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void topBoxClose() {      // ^c
  Serial.println(F("topBoxClose()"));                       // The CLOSE button on the TopBox is being pressed
  tbBtn = 1;                                                // TopBox button invoked this action
  buzz(-100);
  closeRoof();                                              // Try and get the roof to close
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool roofOpen() {
  if (test) return testSw_open;
  return (digitalRead(SW_OPEN));                            // 0 = roof not fully open, 1 = fully open (switch closed)
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool roofClosed() {
  if (test) return testSw_closed;
  return (digitalRead(SW_CLOSED));                          // 0 = roof not fully closed, 1 = fully closed (switch closed)
}
//==================================================================================================================================================
// PARKING TELESCOPE STUFF
//==================================================================================================================================================
char startParking() {      // ^_                            // Park scope, 0 = can't park, 1 = parked, P = parking
  if (cmd == '_'){
    getTerm();
    parkMode = 0;
  }
  Serial.println(F("startParking()"));
  stillParking = 0;
  if (!ee.park) return cmd = '1';                           // If mustn't park, return "parked"

  switch (isParked()) {
    case '1': return cmd = '1'; break;                      // LX200 parked
    case '2': return cmd = '2'; break;                      // LX200 not responding
    default:                    break;
  }

  char buf[34];
  for (byte i = 0; i < 33; i++) buf[i] = 0;                   // Clear buffer

  LX.listen();                                                // Hello LX200
  LX.print("#:Q#:Qn#:Qe#:Qs#:Qw#");                           // Kill all slewing
  LX.print("#:GS#");                                          // Determine park coords. First, ask for sidereal time
  if (LX.readBytesUntil('#', buf, 9) <= 0) return cmd = '0';  // Get sidereal time (RA at meridian), return if no response
  Serial.print("=> Sidereal time: "); Serial.println(buf);
  int h, m, s;
  sscanf(buf, "%02d:%02d:%02d",&h ,&m ,&s);                   // Read in hours, minutes and seconds of RA at meridian
  float ra = h + m / 60.0 - ee.ha;                            // Park RA = meridian RA - hour angle
  if (ra > 23) ra -= 24;                                      // Determine new values for h and m
  if (ra < 0) ra += 24;
  h = int(ra);
  m = int((ra - h) * 60 + 0.5);

  sprintf(buf, "#:Sr%02d:%02d:%02d#:Sd+%02d*00:00#:MS#:\x06#", h, m, s, int(abs(ee.dec))); // Set GO TO coordinates
  if (ee.dec < 0) buf[16] = '-';                                            // Set Dec sign
  Serial.print(F("=> slewing: "));  Serial.println(buf);
  if (itsRaining) LX.print(F("#:Sw8#"));                                    // If raining, move the telescope quickly
  LX.print(buf);                                                            // Slew to GOTO coordinates (parked position)
  for (byte i = 0; i < 33; i++) buf[i] = 0;                                 // Clear buffer
  LX.readBytesUntil('P', buf, 4);                                           // Get any response
  Serial.print(F("=> LX200 slew response - ")); Serial.println(buf);
  TB.listen();                                                              // Thanks LX200.  Hello TopBox
  cmd = (dir == DIR_CLOSE) ? PARK_CLOSE : NOPARK_OPEN;                      // Assume LX200 can start parking
  stillParking = 1;
  if (buf[itsRaining ? 3 : 2] != '0') {                                     // [SW] Sr Sd MS
    cmd = (dir == DIR_CLOSE) ? NOPARK_CLOSE : NOPARK_OPEN;                  // Not '0' means LX200 cannot carry out the slew
    stillParking = 0;
  } else parkingTimer = millis() + PARK_PERIOD;                             // Check on the LX200 while it is parking

  return cmd;
// #:Sr23:59:59#:Sd+90:00:00#:MS#\0
// 0123456789012345678901234567890
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void parkingCheck () {
  if (TB.available()) return;                                   // Quit immediately if TopBox has sent something
  if (millis() > parkingTimer) parking();
  if (parkMode != PARK_OPEN && parkMode != PARK_CLOSE) return;  // If not trying to park the telescope then return
  if (test) {
    if (!lx200_T) {                                             // Abort if 'no connection with LX200'
      Serial.println(F("=> Telescope connection lost"));
      return;
    }
    if (!park_T) return;                                        // Return if 'still parking'
  }
  if (!scopeSafeCheck()) return;

  cmd = 0;
  switch (parkMode) {                                           // It is now safe to move the roof
    case PARK_OPEN:         openRoof();   break;                // Open roof
    case PARK_CLOSE:        closeRoof();  break;                // Close roof
    default: parkMode = 0;                break;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void parking() {                                                  // Determines if telescope is still slewing to home position
  Serial.print(F("parking() =>"));
  Serial.println(parkMode);
  parkingTimer = millis() + 1500;
  char buf[34]; for (byte i = 0; i < 33; i++) buf[i] = 0;         // Prepare read buffer

  if (digitalRead(LX_CXN)) {                                      // Talk to LX200 if connected
    LX.listen();                                                  // Hello LX200
    LX.print(F("#:\x06#"));                                       // Are you parked?
    LX.readBytes(buf, 1);                                         // Get one byte response
    if ((stillParking = (buf[0] != 'L'))) {                       // See if still parking
      buf[16] = 'X';                                              // Mark with 'X' in case no response from LX200
      LX.print("#:D#");                                           // See if the distance bar characters exist
      LX.readBytesUntil('#', buf, 33);
      stillParking = (buf[0] != ' ' || buf[16] != ' ');

      if (!stillParking) {                                        // If no distance bars then telescope has arrived
        LX.print(F("#:Q#:Q#:Q#:Q#:Q#:Q#"));                       // Kill all slewing
        LX.print(F("#:Sw3#"));                                    // Reset LX200's slew speed to 3 degrees/second
        LX.print(F("#:AL#"));                                     // Put LX200 into LAND mode - now parked    
        LX.print(F("#:Q#:Qn#:Qe#:Qs#:Qw#:#:Q#:Q#:Q#:Q#:Q#"));     // Kill all slewing
        Serial.println("TELESCOPE PARKED");
      }
    }
    TB.listen();
  } else stillParking = 1;                                        // Lost connection - assume still parking
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
char isParked() {         // ^p
  getTerm();
  Serial.println(F("isParked()"));
  if (!ee.isParked) cmd = '1';                                    // Ignore isParked? Return 'parked'
  else {
    if (lx200() == '0') {
      parkMode = 0;
      cmd = '2';                                                  // Return '2' if LX200 is not repsonding
      Serial.println(F("=> Telescope not responding"));
      return cmd;
    }
    if (test) {
      cmd = '0' + park_T;
    } else {
      char buf[2];
      buf[0] = buf[1] = 0;
      LX.listen();
      LX.print(F("#:\x06#"));                                       // Get LX200's alignment mode
      LX.readBytes(buf, 1);
      TB.listen();                                                  // Thank you LX200.  Select TopBox
      cmd = (buf[0] == 'L') ? '1' : '0';                            // '1' = parked, '0' = not parked ('L' = LAND mode)
      if (cmd == '1') stillParking = 0;
    }
  }
  Serial.println((cmd == '0') ? F("=> not parked") : F("=> parked"));
  return cmd;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void stopParking () {     // ^Q stop parking, and roof from moving if it is already
  Serial.println(F("\nstopParking()"));
  LX.listen();
  LX.print(F("#:Qn#:Qe#:Qs#:Qw#:Q#:Q#:Q#:Q#:Q#:Q#:Sw3#"));    // Kill all slewing
  TB.listen();
  parkMode = stillParking = 0;
  if (pwm) halt(3);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
char lx200() {            // ^l                             // Sees if LX200 is connected and responding
  if (cmd) getTerm();
  Serial.print(F("lx200() "));
  cmd = '0';                                                // Start off assuming not connected
  if (test){
    cmd = lx200_T + '0';
  } else {
    if (digitalRead(LX_CXN)) {
      char buf[2];
      buf[0] = buf[1] = 0;
      LX.listen();                                          // Select Bluetooth module for the LX200
      LX.print(F("#:\x06#"));                               // Ask for mount mode to see if LX200 responds
      LX.readBytes(buf, 1);
      TB.listen();                                          // Select Bluetooth module for the TopBox
      Serial.print(F("=> Sent #:[6]#\n=> Got: "));
      Serial.println(buf);
      switch (buf[0]) {                                     // If response is A, L or P then LX200 is connected
        case 'A': cmd = '2';  break;                        // 1 = LX200 connected
        case 'L': cmd = '1';  break;
        case 'P': cmd = '3';  break;
        default:  cmd = '0';  break;
      }
    }
  }
  Serial.println( (cmd == '0') ? F("=> LX200 not responding") : F("=> LX200 is responding"));
  return cmd;
}
//==================================================================================================================================================
// MOVING ROOF STUFF
//==================================================================================================================================================
void moveRoof() {
  if (!pwm) {                                                           // See if roof is moving - act if not
    if (button(BTN_OPEN) == PRESSED && button(BTN_CLOSE) != PRESSED) {  // Act if OPEN button is being pressed and CLOSE button is not
      buttonTimer = millis() + 1000;                                    // Wait for button to be pressed for > 1 sec
      while (millis() < buttonTimer) if (button(BTN_OPEN) != PRESSED) return;
      Serial.print(F("moveRoof() => Open button"));
      buzz(500);
      while (button(BTN_OPEN) == PRESSED);                              // Go no further until the button has been released      
      openRoof();                                                       // Start opening the roof
    }

    if (button(BTN_CLOSE)== PRESSED && button(BTN_OPEN) != PRESSED) {   // Act if CLOSE button is being pressed and OPEN button if not
      if (roofClosed()) {                                               // Only if roof is closed can the battery status be indicated
        float v = analogRead(BATTERY) /SENSE_ADJ > BATT_GOOD;           // If > BATTGOOD then battery is good or else it needs charging
        while (button(BTN_CLOSE) == PRESSED) {                          // The LED will show battery status while CLOSE button is pressed
          digitalWrite(LED, v ? HIGH : bitRead(millis(), 7));           // LED is on if battery good, flashing every 128ms = needs charging
        }
        digitalWrite(LED, LOW);
      }
      else {                                                            // Otherwise try closing the roof
        buttonTimer = millis() + 1000;                                  // Wait for CLOSE button to be pressed > 1 sec
        while (millis() < buttonTimer) if (button(BTN_CLOSE) != PRESSED) return;
        Serial.print(F("moveRoof() => Close button"));
        buzz(50);
        while (button(BTN_CLOSE) == PRESSED);                           // Go no further until the button has been released
        closeRoof();                                                    // Start closing the roof
      }
    }
  }
  else {                                                                                // Roof is moving
    if ((button(BTN_OPEN) == PRESSED) || (button(BTN_CLOSE) == PRESSED)) slowHalt(4);   // Abort move if a button has been pressed
  }

  if (pwm) {
    if ((battSensor() / SENSE_ADJ < BATT_BAD) && !itsRaining) {
      halt(0);
      Serial.println(F("Move aborted - no power"));
      return;
    }
    if (!scopeSafeCheck()) {
      halt(11);
      Serial.println(F("Move aborted - scope not safe"));
      return;
    }
    if (millis() >= moveTimer) {
      halt(5);
      timedOut = 1;
      Serial.println(F("Roof move timed out"));
      return;
    }
    if ((dir == DIR_OPEN && roofOpen()) || (dir == DIR_CLOSE && (roofClosed()))) halt(6);
  }
  if (isr) {
    moveTimer = parkingTimer = parkMode = tbBtn = pwm = isr = 0;
    maxSpeed = ee.maxSpeed;
    if (DIR == DIR_CLOSE) rainSensor(0);                // Switch off the rain sensor to save power
  }

  //if (!pwm || pwm >= maxSpeed) if (stillParking) if (millis() > parkingTimer) parking();
  motorControl();
  if (pwm) {                                            // Roof is moving
    if (pwm < maxSpeed) {                               // Ramp up the motor speed
      if (millis() >= rampTimer) {
        pwm += PWM_INC;
        if (pwm > maxSpeed) pwm = maxSpeed;
        rampTimer = millis() + PWM_DELAY;
      }
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void motorControl() {
  if (pwm < pwmFlag) {
    if (pwm > 1) Serial.print(F("motorControl()\nSlowing down to 0 from ")); Serial.println(pwmFlag);
    pwmFlag = -1;
  }
  if (pwm) {
    if (pwm == 1 && pwmFlag == 0) {
      Serial.print("ISR activated - "); Serial.print((dir == DIR_CLOSE) ? F("CLOSE - ") : F("OPEN - "));
      Serial.println(digitalRead((dir == DIR_CLOSE) ? SW_CLOSED : SW_OPEN));
      Serial.print(F("motorControl()\nRamping up to ")); Serial.println(maxSpeed);
      noInterrupts();
      isr = 0;
      attachInterrupt(digitalPinToInterrupt(dir == CLOSED ? SW_CLOSED : SW_OPEN), stopISR, HIGH);
      interrupts();
    }
    if (pwm >= maxSpeed && pwmFlag > 0) {
      Serial.print(F("motorControl()\nTop speed reached: ")); Serial.println(pwm);
            pwm = maxSpeed;
            pwmFlag = -1;
    }
    if (ee.battery && !startedRaining()) {
      if (battSensor() / SENSE_ADJ <= BATT_BAD) {
        Serial.println(F("motorControl()\nNo power from battery - motor stopped")); // Don't want Arduino to power the motor!
        pwm = 0;
        pwmFlag = -1;
      }
    }
    if (!scopeSafeCheck()) {
      buzz(-100);
      if(!tbBtn) {
        pwm = 0;
        pwmFlag = -1;
      }
      Serial.println(F("motorControl()\nTelescope not safe"));
    }
    if (pwmFlag >= 0) pwmFlag = pwm;
  } else {                                                                          // This is probably overkill
    detachInterrupt(digitalPinToInterrupt(SW_CLOSED));
    detachInterrupt(digitalPinToInterrupt(SW_OPEN));
    isr = 0;
  }
  digitalWrite(DIR, dir);                                                           // Set motor direction
  analogWrite(PWM, test ? 0 : pwm);                                                 // Adjust motor speed
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void halt(byte h) {       // ^H
  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(SW_CLOSED));
  detachInterrupt(digitalPinToInterrupt(SW_OPEN));
  isr = 0;
  interrupts();
  if (pwm) pwmFlag = pwm;
  moveTimer = safetyTimer = parkingTimer = tbBtn = pwm = 0;   // moveTimer is reset as roof is no longer moving
  if (stillParking) stopParking();
  motorControl();
//  getTerm();
  maxSpeed = ee.maxSpeed;
  Serial.print(F("halt(")); Serial.print(h); Serial.println(')');
  Serial.println(F("=> Stopped "));
  buzz(-500);                                                 // Start buzzing
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void slowHalt(byte h) {   // ^h
  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(SW_CLOSED));
  detachInterrupt(digitalPinToInterrupt(SW_OPEN));
  isr = 0;
  interrupts();
  Serial.print(F("slowHalt(")); Serial.print(h); Serial.println(')');
  pwmFlag = pwm;
  while (pwm) {
    pwm --;                                               // Slow down a bit
    if (pwm < 0) pwm = 0;
    if (dir == DIR_OPEN && roofOpen()) pwm = 0;           // Roof is fully open - stop quickly
    if (dir == DIR_CLOSE && roofClosed()) pwm = 0;        // Roof is fully closed - stop quickly
    motorControl();
    delay(PWM_HALT);                                      // Wait a bit before looping
  }
  maxSpeed = ee.maxSpeed;
  getTerm();
  if (stillParking) stopParking();
  moveTimer = safetyTimer =  pwmFlag = tbBtn = 0;         // moveTimer is reset as roof is no longer moving
  Serial.println(F("=> Stopped "));
  buzz(-500);                                             // Start buzzing
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void stopISR() {                                      // Come here if either the SW_OPEN or SW_CLOSED switch has been activated
  noInterrupts();
  if (!digitalRead(dir == DIR_CLOSE ? SW_CLOSED : SW_OPEN)){
    interrupts();
    return;
  }
  analogWrite(PWM, 0);                                // Stop the motor immediately
  detachInterrupt(digitalPinToInterrupt(SW_CLOSED));
  detachInterrupt(digitalPinToInterrupt(SW_OPEN));
  interrupts();
  isr = 1;                                            // Mark that interrupt has been triggered
  pwm = 0;
  Serial.print(F("ISR triggered halt: ")); Serial.println((dir == DIR_CLOSE) ? "CLOSE switch" : "OPEN switch");
  for(byte i = 0; i < 6; i++){
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void override(){
  digitalWrite(LED,((millis() & B10000) > 0));                           // Sawtooth flash = all is OK. Cycles every 4096ms 
  byte pwm = 0;
  while(1) {
    if (button(BTN_CLOSE) && !digitalRead(SW_CLOSED)) {
      while(!button(BTN_OPEN) && !digitalRead(SW_CLOSED)) {
        pwm ? delay(10) : buzz(50);
        if (pwm < 191) pwm++;
        digitalWrite(DIR, DIR_CLOSE);
        analogWrite(PWM, pwm);
      }
    }

    if(pwm) {
      pwm = 0;
      analogWrite(PWM, pwm);
      buzz(50);
    }

    if (button(BTN_OPEN) && !digitalRead(SW_OPEN)) {
      while(!button(BTN_CLOSE) && !digitalRead(SW_OPEN)) {
        pwm ? delay(10) : buzz(50);
        if (pwm < 191) pwm++;

        digitalWrite(DIR, DIR_OPEN);
        analogWrite(PWM, pwm);
      }
    }

    if(pwm) {
      pwm = 0;
      analogWrite(PWM, pwm);
      buzz(50);
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void scopeSafeStatus(byte v) { // ^T Respond to TopBox's announcement's of scope's safety
  Serial.print(F("scopeSafeStatus() => "));
  scopeSafe = v;
  if (!ee.safe) {
    scopeSafe = 1;
  }
  else if(test) scopeSafe = safe_T;
  Serial.println(scopeSafe ? F("Scope is safe") : F("SCOPE NOT SAFE"));
  safetyTimer = millis() + 5000;
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
int scopeSafeCheck() {
  if (!ee.safe) return 1;      // Ignore scope's safety - return 'safe'
  if (test) return safe_T;
  return scopeSafe;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void status() {           // ^s
  getTerm();
  Serial.println(F("status()"));
  if (parkMode) {
    switch (parkMode){
      case PARK_OPEN: cmd = PARK_OPEN; break;
      case PARK_CLOSE: cmd = PARK_CLOSE;  break;
      case LXCXN_OPEN: cmd = LXCXN_OPEN; break;
      case LXCXN_CLOSE: cmd = LXCXN_CLOSE; break;
    }
    return;
  }
  if (roofOpen() && !pwm) {
    cmd = OPENED;                                               // Roof is fully opened
    return;
  }
  if (roofClosed() && !pwm) {
    cmd = CLOSED;                                               // Roof is fully closed
    return;
  }
  if (timedOut) {
    cmd = (dir == DIR_OPEN ? TIME_OPEN : TIME_CLOSE);           // Move timed out - obstruction?
    return;
  }
  if (battSensor() / SENSE_ADJ < BATT_BAD) {
    cmd = (dir == DIR_OPEN ? BATT_OPEN : BATT_CLOSE);            // Battery voltage no longer OK
    return;
  }
  if (startedRaining() || itsRaining) {
    cmd = (dir == DIR_OPEN ? RAIN_OPEN : RAIN_CLOSE);            // Raining while moving roof
    return;
  }
  if (!scopeSafe) {
    cmd = (dir == DIR_OPEN ? SAFE_OPEN : SAFE_CLOSE);            // Telescope is not in a safe position
    return;
  }
  if (pwm) cmd = (dir == DIR_OPEN ? OPENING : CLOSING);          // Depending on direction, roof is either opening or closing
  else cmd = (dir == DIR_OPEN ? STOP_OPEN : STOP_CLOSE);         // Assume roof is not moving and is not fully open or closed
}
/*  C = CLOSED      Closed
    c = CLOSING     Closing
    O = OPEN        Open
    o = OPENING     Opening
    h = STOP_OPEN   Stopped and partly opened
    H = STOP_CLOSE  Stopped and partly closed
    p = PARK_OPEN   Telescope parking, when complete roof will open
    P = PARK_CLOSE  Telescope parking, when complete roof will close
    b = BATT_OPEN   Battery voltage bad, trying to open
    B = BATT_CLOSE  Battery voltage bad, trying to close
    t = TIME_OPEN   Timed out while opening
    T = TIME_CLOSE  Timed out while closing
    r = RAIN_OPEN   Raining while opening
    R = RAIN_CLOSE  Raining while closing
    s = SAFE_OPEN   Telescope no longer safe while opening
    S = SAFE_CLOSE  Telescope no longer safe while closing
    l = LXCXN_OPEN  Telescope not repsonding while parking to open
    L = LXCXN_CLOSE Telescepe not responding while parking to close
    */
//--------------------------------------------------------------------------------------------------------------------------------------------------
void swStatus() {         // ^w
  cmd = 'M';                                // Middle/Moving - both switches are open

  if (roofOpen()) cmd = 'O';                // Open Switch is closed
  if (roofClosed()) cmd = 'C';              // Closed Switch is closed
  // if (digitalRead(SW_OPEN)) cmd = 'O';      // Open Switch is closed
  // if (digitalRead(SW_CLOSED)) cmd = 'C';    // Closed Switch is closed
}
//==================================================================================================================================================
// BATTERY STUFF
//==================================================================================================================================================
void getBattery() {       // ^b
  getTerm();
  float v = battSensor();                                    // Read battery voltage - probably won't be accurate if roof is moving
  Serial.print(F("getBattery() => "));
  Serial.println(v);
  v /= SENSE_ADJ;
  Serial.print(F("=> "));
  Serial.println(v);
  if (serial) TB.print(v, 1);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
int battSensor() {
  if (!ee.battery) return 780;  // Ignore battery sensor - return 'good'
  if (test) return batt_T;
  return analogRead(BATTERY);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool batteryCheck() {
  return ((battSensor() / SENSE_ADJ) > BATT_BAD);
}
//==================================================================================================================================================
// RAIN SENSOR STUFF
//==================================================================================================================================================
void actIfRaining() {
  if (!ee.rain) return;               // Return if rain is not to be detected
  if (itsRaining) return;             // Return if we know it's already raining
  if (roofClosed()) return;           // Return if roof is closed - who cares if it's raining!
  if (millis() < rainTimer) return;   // Return if rain sensor is still booting up
  if (!startedRaining()) return;      // Return if not started raining

  Serial.println(F("actIfRaining()"));
  buzz(-5000);
  maxSpeed = 255;                     // Move the roof quickly
  cmd = 0;
  closeRoof();
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void rainSensor(byte m) { // ^R     0 = turn off, 1 = turn on, 2 = restart (off/on), just turn on rain sensor
  if (m == rainSensorPwr)  return;
  Serial.print(F("rainSensor() => "));
  switch (m) {
    case 0:     Serial.println(F("Off"));         break;
    case 1:     Serial.println(F("On"));          break;
    case 2:     Serial.println(F("Restart"));     break;
    case 3:     Serial.println(F("Force on"));    break;
    default:                                      break;
  }
  if (m < 3) if (cmd == 'R' || cmd == 'e')  getTerm();
  if (m == 0 || m == 2) {                         // Turn sensor off
    digitalWrite(RG9_VCC1, LOW);
    digitalWrite(RG9_VCC2, LOW);
    rainTimer = 0;
    if (m == 2) delay(1000);
    rainSensorPwr = 0;
  }
  if (m > 0) {                                    // Turn sensor on
    if (digitalRead(RG9_VCC1) == LOW) {
      rainTimer = millis() + 11000;               // If not actually on, start 11 second timer to allow time for rain sensor to initialise
      itsRaining = 0;
      digitalWrite(RG9_VCC1, HIGH);
      digitalWrite(RG9_VCC2, HIGH);
      rainSensorPwr = 1;    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getRainSensor() {    // ^r
  getTerm();
  Serial.print(F("getRainSensor() =>"));
  cmd = (char)(startedRaining() + '0');
  Serial.println(cmd == '0' ?  F("Dry") : F("Wet"));
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool startedRaining() {
  if (!ee.rain) return (itsRaining = 0);        // Ignore rain sensor - return 'dry'
  if (test) return (itsRaining = rain_T);       // Return test mode value
  return (itsRaining = !digitalRead(RG9));      // RG9 = 0 means it's raining so return 1 if so, 0 if not.
}
//==================================================================================================================================================
// PROPERTIES
//==================================================================================================================================================
void setParameters() {    // ^U
  Serial.print(F("setParameters() => "));
  char buf[3] = {0, 0, 0};
  serial ? TB.readBytesUntil('$', buf, 2) : Serial.readBytesUntil('$', buf, 2);
  Serial.println(buf[0]);
  switch (buf[0]) {
    case 'P': ee.park = 1;                        break;      // UP
    case 'p': ee.park = 0;                        break;      // Up
    case 'I': ee.isParked = 1;                    break;      // UI
    case 'i': ee.isParked = 0;                    break;      // Ui
    case 'S': ee.safe = 1;                        break;      // US
    case 's': ee.safe = 0;                        break;      // Us
    case 'B': ee.battery = 1;                     break;      // UB
    case 'b': ee.battery = 0;                     break;      // Ub
    case 'R': ee.rain = 1;                        break;      // UR
    case 'r': rainSensor(cmd = ee.rain = 0);      break;      // Ur
    default: cmd = '%';                           break;  
  }
  if (cmd != '%') writeEE();
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getParameters() {    // ^u
  Serial.print(F("getParameters()\n=> "));
  char buf[3] = {0, 0, 0};
  serial ? TB.readBytesUntil('$', buf, 2) : Serial.readBytesUntil('$', buf, 2);
  Serial.println(buf[0]);
  switch (buf[0]) {
    case 'P': buf[1] = '0' + ee.park;       break;    // uP
    case 'I': buf[1] = '0' + ee.isParked;   break;    // uI
    case 'S': buf[1] = '0' + ee.safe;       break;    // uS
    case 'B': buf[1] = '0' + ee.battery;    break;    // uB
    case 'R': buf[1] = '0' + ee.rain;       break;    // uR
    case 'A': getAllParameters();           break;    // Get all parameters
    default: cmd = '%';                     break;
  }
  if (cmd != '%') {
    if (serial) {
      TB.print(buf[1]);
    }
    Serial.print(buf[1]);
    cmd = 0;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getAllParameters() {
  TB.print(ee.ha); TB.print(' ');
  TB.print(ee.dec); TB.print(' ');
  TB.print(ee.park); TB.print(' ');
  TB.print(ee.isParked); TB.print(' ');
  TB.print(ee.safe); TB.print(' ');
  TB.print(ee.battery); TB.print(' ');
  TB.print(ee.rain); TB.print(' ');
  TB.print(ee.movePeriod); TB.print(' ');
  TB.print(ee.maxSpeed); TB.print(' ');
  TB.println(VERSION);

  Serial.print(ee.ha); Serial.print(' ');
  Serial.print(ee.dec); Serial.print(' ');
  Serial.print(ee.park); Serial.print(' ');
  Serial.print(ee.isParked); Serial.print(' ');
  Serial.print(ee.safe); Serial.print(' ');
  Serial.print(ee.battery); Serial.print(' ');
  Serial.print(ee.rain); Serial.print(' ');
  Serial.print(ee.movePeriod); Serial.print(' ');
  Serial.print(ee.maxSpeed); Serial.print(' ');
  Serial.println(VERSION);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void setDEC() {           // ^D
  Serial.print(F("setDEC() => "));
  ee.dec =  serial ? TB.parseFloat() : Serial.parseFloat(); // Get scope's home position's Dec
  getTerm();
  Serial.println(ee.dec);
  writeEE();                                                // Save this to EEPROM
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getDEC() {           // ^d
  getTerm();
  Serial.print(F("getDEC() => "));
  if (serial) TB.print(ee.dec);                             // Return the stored scope's home position's Dec
  Serial.println(ee.dec);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void setHA() {            // ^A
  Serial.println(F("setHA() => "));
  ee.ha = serial ? TB.parseFloat() : Serial.parseFloat();   // Get scope's home position's Hour Angle
  getTerm();
  Serial.print(ee.ha); Serial.println('$');
  writeEE();                                                // Save this to EEPROM
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getHA() {            // ^a
  getTerm();
  Serial.print(F("getHA() => "));
  if (serial) TB.print(ee.ha);                                // Return the stored scope's parked Hour Angle
  Serial.print(ee.ha);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void setMoveTimer() {     // ^M
  Serial.print(F("setMoveTimer() => "));
  ee.movePeriod =  serial ? TB.parseInt() : Serial.parseInt(); // Get scope's home position's Dec
  getTerm();
  Serial.println(ee.movePeriod);
  writeEE();                                                  // Save this to EEPROM
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getMoveTimer() {     // ^m
  getTerm();
  Serial.print(F("getMoveTimer() => "));
  if (serial) {
    TB.print(ee.movePeriod);                                  // Return the stored scope's home position's Dec
  }
  Serial.print(ee.movePeriod);
  cmd = 0;  
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void setMaxSpeed() {      // ^X
  Serial.print(F("setMaxSpeed() => "));
  ee.maxSpeed = maxSpeed =  serial ? TB.parseInt() : Serial.parseInt(); // Get scope's home position's Dec
  getTerm();
  Serial.println(ee.maxSpeed);
  writeEE();                                                            // Save this to EEPROM
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getMaxSpeed() {      // ^x
  getTerm();
  Serial.print(F("getMaxSpeed() => "));
  if (serial) TB.print(ee.maxSpeed);                          // Return the maximum speed for roof (0 to 255)
  Serial.print(ee.maxSpeed);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void getMovePercent() {   // ^g
  Serial.print(F("getMoveProg() => "));
  long v = moveTimer - millis();
  if (v < 0) v = 0;
  v = 100 - v / (10 * ee.movePeriod);
  getTerm();
  if (serial) TB.print(v);
  Serial.print(v);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void version() {          // ^V
  getTerm();
  if (serial) TB.print(VERSION);
  Serial.println(VERSION);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void variables() {        // ^v
  getTerm();
  Serial.println(F("VARIABLES\n========="));
  Serial.print(F("cmd:            ")); Serial.println(cmd);
  Serial.print(F("timedOut:       ")); Serial.println(timedOut);
  Serial.print(F("pwm:            ")); Serial.println(pwm);
  Serial.print(F("dir:            ")); Serial.println(dir);
  Serial.print(F("pwmFlag:        ")); Serial.println(pwmFlag);
  Serial.print(F("scopeSafe:      ")); Serial.println(scopeSafe);
  Serial.print(F("parkMode:       ")); Serial.println(parkMode);
  Serial.print(F("stillParking:   ")); Serial.println(stillParking);
  Serial.print(F("itsRaining:     ")); Serial.println(itsRaining);
  Serial.print(F("millis():       ")); Serial.println(millis());
  Serial.print(F("moveTimer:      ")); Serial.println(moveTimer);
  Serial.print(F("parkTimer:      ")); Serial.println(parkingTimer);
  Serial.print(F("buzzTimer:      ")); Serial.println(buzzerTimer);
  Serial.print(F("rainTImer:      ")); Serial.println(rainTimer);
  Serial.print(F("rampTimer:      ")); Serial.println(rampTimer);
  Serial.print(F("flashTimer:     ")); Serial.println(flashTimer);
  Serial.print(F("buttonTimer:    ")); Serial.println(buttonTimer);
  Serial.print(F("serial:         ")); Serial.println(serial);
  Serial.print(F("flashType:      ")); Serial.println(flashType);
  Serial.print(F("flashPos:       ")); Serial.println(flashPos);
  Serial.print(F("tbBtn:          ")); Serial.println(tbBtn);
  Serial.print(F("ee.ha:          ")); Serial.println(ee.ha, 1);
  Serial.print(F("ee.dec:         ")); Serial.println(ee.dec);
  Serial.print(F("ee.movePeriod:  ")); Serial.println(ee.movePeriod);
  Serial.print(F("ee.maxSpeed:    ")); Serial.println(ee.maxSpeed);
  Serial.print(F("ee.park:        ")); Serial.println(ee.park);
  Serial.print(F("ee.isParked:    ")); Serial.println(ee.isParked);
  Serial.print(F("ee.safe:        ")); Serial.println(ee.safe);
  Serial.print(F("ee.rain:        ")); Serial.println(ee.rain);
  Serial.print(F("ee.battery      ")); Serial.println(ee.battery);
  Serial.print(F("test:           ")); Serial.print(!test); Serial.println(test);
  Serial.print(F("sw_open:        ")); Serial.println(sw_open);
  Serial.print(F("sw_closed:      ")); Serial.println(sw_closed);
}
//==================================================================================================================================================
// HARDWARE STUFF
//==================================================================================================================================================
void serviceLED() {
  const int flashLED[5][8] = {
    {125, 125, 0, 0, 0, 0, 0, 1000},                                      // 1 flash   = TopBox Bluetooth not connected
    {125, 125, 125, 125, 0, 0, 0, 1000},                                  // 2 flashes = LX200  Bluetooth not connected
    {125, 125, 125, 125, 125, 125, 0, 1000},                              // 3 flashes = Neither Bluetooth connected
    {125, 125, 125, 125, 125, 125, 125, 1000},                            // 4 Flashes = Roof timed out
    {2750, 250,0 ,0 ,0 ,0, 0, 0} };                                       // Long flashes = rain detected

  flashType = 0;                                                          // Assume all is OK
  if (!digitalRead(TB_CXN)) flashType = 1;                                // TopBox Bluetooth not connected - flash once
  if (!digitalRead(LX_CXN)) flashType += 2;                               // LX200 Bluetooth not connected  - flash twice
  if (timedOut) flashType = 4;                                            // Roof timed-out in its move - flash continuously
  if (itsRaining) flashType = 5;                                          // Oh no! It rained - long flashes
  if (flashType) {
    if (millis() >= flashTimer) {
      flashType --;
      if (flashPos >= 8) flashPos = 0;                                    // Cycle through the flash sequence
      if (flashLED[flashType][flashPos]) digitalWrite(LED, (flashPos ^1) & 1);
      flashTimer = millis() + flashLED[flashType][flashPos];
      flashPos++;
    }
  }
  else analogWrite(LED, byte(millis() / 16)/4);                           // Sawtooth flash = all is OK. Cycles every 4096ms 
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void buzz(long d) {               // 0 = turn off buzzer, -1 = continuous beep, < -1 async timer beep, > 0 wait for beep to finish
  Serial.print(F("buzz() => "));
  Serial.println(d);
  digitalWrite(BUZZER, LOW);      // Switch off buzzer
  if (!d) return;                 // Return immediately if d is 0 (buzzer is turned off)
  digitalWrite(BUZZER, HIGH);     // Start buzzer
  if (d == -1) return;            // Return immediately - a continuous beep has been started
  if (d > 0) {                    // Beep for d milliseconds
    delay(d);                     // Wait for it to be done
    digitalWrite(BUZZER, LOW);    // Turn off the buzzer
  } 
  else {
    buzzerTimer = millis() - d;   // Set timer of asychronous beep
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
bool button(int btn) {
  byte v = digitalRead(btn);                                              // Get button status
  delay(20);                                                              // Soak up debounce
  return (digitalRead(btn) + v) == 2;                                     // Returns 1 if pressed, 0 if not
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void hardware() {         // ^~
  getTerm();
  Serial.println(F("ARDUINO'S PINS\n=============="));
  float v;
  Serial.print(F("D2  SW_CLOSED: "));Serial.println(digitalRead(SW_CLOSED));
  Serial.print(F("D3  SW_OPEN:   "));Serial.println(digitalRead(SW_OPEN));
  Serial.print(F("D4  LX_RXD:    "));Serial.println(digitalRead(LX_RXD));
  Serial.print(F("D5  LX_TXD:    "));Serial.println(digitalRead(LX_TXD));
  Serial.print(F("D6  LX_CXN:    "));Serial.println(digitalRead(LX_CXN));
  Serial.print(F("D7  TB_RXD:    "));Serial.println(digitalRead(TB_RXD));
  Serial.print(F("D8  TB_TXD:    "));Serial.println(digitalRead(TB_TXD));
  Serial.print(F("D9  TB_CXN:    "));Serial.println(digitalRead(TB_CXN));
  Serial.print(F("D10 LED:       "));Serial.println(digitalRead(LED));
  Serial.print(F("D11 PWM:       "));Serial.print(digitalRead(PWM)); Serial.print(' '); Serial.println(pwm);
  Serial.print(F("D12 DIR:       "));Serial.print(digitalRead(DIR)); Serial.print(' '); Serial.println(dir);
  Serial.print(F("D13 BUZZER:    "));Serial.println(digitalRead(BUZZER));

  Serial.print(F("A0  BATTERY:   "));Serial.print(digitalRead(A0));Serial.print(' ');Serial.print(v = analogRead(A0), 0);Serial.print(F(" => "));Serial.println(v / SENSE_ADJ, 1);
  Serial.print(F("A1  BTN_OPEN:  "));Serial.print(digitalRead(A1));Serial.print(' ');Serial.println(analogRead(A1));
  Serial.print(F("A2  BTN_CLOSE: "));Serial.print(digitalRead(A2));Serial.print(' ');Serial.println(analogRead(A2));
  Serial.print(F("A3  RG9:       "));Serial.print(digitalRead(A3));Serial.print(' ');Serial.println(digitalRead(A3));
  Serial.print(F("A4  RG9_VCC1:  "));Serial.print(digitalRead(A4));Serial.print(' ');Serial.println(digitalRead(A4));
  Serial.print(F("A5  RG9_VCC1:  "));Serial.print(digitalRead(A5));Serial.print(' ');Serial.println(digitalRead(A5));
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void hardwareTest() {
  byte sensorFlag = 1;
  while(1) {
    digitalWrite(LED, digitalRead(RG9));
    if (!digitalRead(RG9)) {
      if (sensorFlag) {
        buzz(5000);
        sensorFlag = 0;
      }
    }
    if (digitalRead(RG9)) {
      if (!sensorFlag) {
        sensorFlag = 1;
        buzz(1000);
      }
    }
    while (button(BTN_OPEN)) {   // 1 flash and beep
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(1000);
    }
    while(button(BTN_CLOSE)) {   // 2 flashes and beeps
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(1000);
    }
    while(digitalRead(SW_OPEN)) {  // 3 flashes and beeps
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(1000);    
    }
    while(digitalRead(SW_CLOSED)) {  // 4 flashes and beeps
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      buzz(10);
      delay(200);
      digitalWrite(LED, LOW);
      delay(1000);    
    }
  }
}
//==================================================================================================================================================
// EEPROM STUFF
//==================================================================================================================================================
void readEE() {
  Serial.println(F("readEE()"));
  if (EEPROM[0] == 'R'  && EEPROM[1] == 'B') {            // See if data has been stored before
    EEPROM.get(2, ee);                                    // If so the update the stored data as necessary
  }
  else formatEE();
  maxSpeed = ee.maxSpeed;
  if (!ee.rain) rainSensor(0);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void formatEE() {
    Serial.println(F("formatEE"));
    ee.ha = 0.0;
    ee.dec = -35;
    ee.movePeriod = MOVE_PERIOD;
    ee.maxSpeed = maxSpeed = PWM_MAX;
    ee.park = 1;                                          // Ensure telescope should park
    ee.isParked = 1;                                      // Ensure is parked
    ee.battery = 1;                                       // Ensure scope is safe
    ee.rain = 1;                                          // Enusre rain sensor is monitored
    ee.battery = 1;                                       // Ensure there is enough power
    ee.ha = PARK_HA;                                      // Use default parked HA
    ee.dec = PARK_DEC;                                    // Use default pared DEC
    writeEE();                                            // Format EEPROM and write data
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void writeEE() {
  Serial.println(F("writeEE()"));
  EEPROM.put(0, 'R');                                     // Write "RB" (short for RoofBuddy) as the header
  EEPROM.put(1, 'B');
  EEPROM.put(2, ee);                                      // Write the data
}
//===================================================================================================================================================
// TEST MODE STUFF
//===================================================================================================================================================
void testStatus() {
  Serial.println(F("TEST STATUS\n-----------"));
  Serial.print(F("test:      ")); Serial.println(test ? F("Test mode") : F("Live mode"));
  Serial.print(F("sw_closed: ")); Serial.println(sw_closed);
  Serial.print(F("sw_open:   ")); Serial.println(sw_open);
  Serial.print(F("pwm:       ")); Serial.println(pwm);
  Serial.print(F("dir:       ")); Serial.println(dir == DIR_OPEN ? F("Open") : F("Close"));
  Serial.print(F("batt_T:    ")); Serial.println((batt_T / SENSE_ADJ) > BATT_BAD ? F("Good") : F("Bad"));
  Serial.print(F("rain_T:    ")); Serial.println(rain_T ? F("Wet") :  F("Dry"));
  Serial.print(F("park_T:    ")); Serial.println(park_T ? F("Parked") : F("Not parked"));
  Serial.print(F("safe_T:    ")); Serial.println(safe_T ? F("Safe") : F("Not safe"));
  Serial.print(F("lx200_T:   ")); Serial.println(lx200_T ? F("Connected") : F("Disconnected"));
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void roofH () {           // Z-   Test mode: move aborted - roof half open
  Serial.println(F("roofH()"));
  sw_open = sw_closed = 0;
  testSw_open = testSw_closed = 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void roofC() {            // ZC   Test mode: roof closed
  Serial.println(F("roofC()"));
  sw_open = testSw_closed = cmd = 0;
  sw_closed = testSw_open = 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void roofO() {            // ZO   Test mode: roof open
  Serial.println(F("roofO()"));
  sw_open = testSw_closed = 1;
  sw_closed = testSw_open = cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void battT(byte v) {      // ZB (0) bad, Zb (1) good
  Serial.print(F("battT() => "));
  batt_T = v ? 780 : 304;
  Serial.println(float(batt_T) / SENSE_ADJ, 1);
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void rainT(byte v) {      // Zr (0) wet  ZR (1) dry
  Serial.print(F("rainT() => "));
  rain_T = v ? 1023 : 0;
  Serial.println(rain_T > 511 ? F("Dry") : F("Wet"));
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void parkT(byte v) {      // ZP (0) not parked, Zp (1) parked
  Serial.print(F("parkT() => "));
  park_T = v;
  Serial.println(park_T ? F("Parked") : F("Not parked"));
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void safeT(byte v) {      // Zs (0) not safe, ZS (1) safe
  Serial.println(F("safe() => "));
  scopeSafe = safe_T = v;
  Serial.println(scopeSafe ? F("Safe") : F("Not safe"));
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void lx200T(byte v) {     // Zl (0) not connected, ZL (1) connected
  Serial.println(F("lx200T()"));
  lx200_T = v;
  Serial.println(lx200_T ? F("Connected") : F("Disconnected"));
  cmd = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
void lx200cmd() {         // ^:
  getTerm();
  Serial.print(F("lx200cmd() "));                         // Sends LX200 command and returns any response from the LX200
  cmd = '%';                                              // Assume this is going fail
  char buf[34];
  for (byte i = 0; i <= 33; i++) buf[i] = 0;              // Clear the buffer
  if (!Serial.readBytesUntil('#', buf, 33)) return;       // Get command for the LX200 into the buffer
  Serial.print(F("=> :"));
  Serial.print(buf);
  Serial.println('#');
  if (!digitalRead(LX_CXN)) return;                       // Quit if no Bluetooth connection with LX200
  LX.listen();                                            // Now get the LX200's attention
  LX.print(F("#:")); LX.print(buf); LX.print('#');        // Send command and terminator
  for (byte i = 0; i <= 33; i++) buf[i] = 0;              // Clear the buffer
  LX.readBytesUntil('#', buf, 33);                        // Get LX200 response, if any
  TB.listen();                                            // Thank you LX200.  Now listen to the TopBox again
  if (buf[0]) {                                           // If there is a repsonse then send it to the TopBox
    Serial.print(F("=>")); Serial.print(buf); Serial.println('#');
    if (serial) TB.print(buf);
    cmd = 0;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
