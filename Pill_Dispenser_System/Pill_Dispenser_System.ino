// #include <DS3231.h>
#include <AccelStepper.h>

#define HALFSTEP 8

#include <Stepper.h>
#include <SoftwareSerial.h> 
#include <HCSR04.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC

// DS3232RTC myRTC;
// For some reason, other pins doesn't work.
SoftwareSerial sim800LSerial(10,9);  //RX/TX  

DS3232RTC rtc;

// Defines the number of steps per rotation
// const int stepsPerRevolution = 2048;
const int endPoint = 4096;

// Initialize steppers
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence

#define d1_1  22     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define d1_2  24     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define d1_3  26    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define d1_4  28    // IN4 on ULN2003 ==> Orange on 28BYJ-48

#define d2_1  30     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define d2_2  32     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define d2_3  34    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define d2_4  36    // IN4 on ULN2003 ==> Orange on 28BYJ-48

#define d3_1  38     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define d3_2  40     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define d3_3  42    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define d3_4  44    // IN4 on ULN2003 ==> Orange on 28BYJ-48

#define d4_1  23     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define d4_2  25     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define d4_3  27    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define d4_4  29    // IN4 on ULN2003 ==> Orange on 28BYJ-48

#define d5_1  31     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define d5_2  33     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define d5_3  35    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define d5_4  37    // IN4 on ULN2003 ==> Orange on 28BYJ-48

AccelStepper dispenser1(HALFSTEP, d1_1, d1_3, d1_2, d1_4);
AccelStepper dispenser2(HALFSTEP, d2_1, d2_3, d2_2, d2_4);
AccelStepper dispenser3(HALFSTEP, d3_1, d3_3, d3_2, d3_4);
AccelStepper dispenser4(HALFSTEP, d4_1, d4_3, d4_2, d4_4);
AccelStepper dispenser5(HALFSTEP, d5_1, d5_3, d5_2, d5_4);


// Initialize ultrasonic sensors
#define TANK_TRIG_PIN 2
#define TANK_ECHO_PIN 3
#define CUP_TRIG_PIN 4
#define CUP_ECHO_PIN 5
UltraSonicDistanceSensor tankSensor(TANK_TRIG_PIN, TANK_ECHO_PIN);
UltraSonicDistanceSensor cupSensor(CUP_TRIG_PIN, CUP_ECHO_PIN);

#define WATER_PUMP_PIN 6
#define LED_PIN 7
#define BUZZER_PIN 8

// GLOBAL VARS

String destNumber = "AT+CMGS=\"+639817526118\""; // CHANGE THE NUMBER in +63XXXXXXXXXX format
// String destNumber = "AT+CMGS=\"+639086472754\""; // CHANGE THE NUMBER in +63XXXXXXXXXX format

// const int REVOLUTIONS = 2; // How many revolutions per dispense. For pill-empty-pill, 2 revolutions. For pill-pill, 1 revolution.

const int MAX_NUM_PILLS = 10;
const int MAX_NUM_SCHEDULES = 3;

const int SNOOZE_INTERVAL = 1;
const int MAX_SNOOZES = SNOOZE_INTERVAL * 5;

// Change this as desired
const int BLINK_TIMES = 5;
const int BLINK_DELAY = 100;

int snooze;

char incomingByte; 
String inputString;

int skipFlag;
int tankSkipFlag;
int dispensedFlag;

int pillCount[5];
tmElements_t schedules[5][MAX_NUM_SCHEDULES];
int numSchedules[5];


// Change this
String pillNames[] = {"1", "2", "3", "4", "5"};

tmElements_t snoozeTimer;
tmElements_t lastSnooze;

// Tank level, in cm
float tank_level = 0;
float tank_level_previous = 0;
float tankLevelHigh = 3.0; // 3cm gap between sensor and top of water
float tankLevelLow = 15.0; // 10cm gap, calibrate depending on tank height, CHANGE THIS

// Cup distance, in cm
float cup_distance = 0;
float cupThreshold = 3.0; // 3cm gap between sensor and cup, CHANGE THIS

int pumpBuffer = 40000; // pump for 40 seconds, CHANGE THIS

void firstBoot(){
  Serial.println("Sending first boot message to registered number.");
  sim800LSerial.println("AT+CMGF=1");
  updateSerial();
  sim800LSerial.println(destNumber);
  updateSerial();
  sim800LSerial.println("Greetings, user!"); // Personalize
  sim800LSerial.println(" ");
  sim800LSerial.println("Pill dispensing system is up and running.");
  sim800LSerial.println("Since the system has just been booted, the number of pills in its memory have been wiped");
  sim800LSerial.println("Initialize the number of pills in storage by sending me a message!");
  sim800LSerial.println(" ");
  sim800LSerial.println("To SET the amount of pills in all dispensers, send \"SET ALL, <amount>\"");
  sim800LSerial.println("To SET the amount of pills in a specific dispenser, send \"SET P<number>, <amount>\"");
  sim800LSerial.println("To ADD an amount of pills in all dispensers, send \"ADD ALL, <amount>\"");
  sim800LSerial.println("To ADD an amount of pills in a specific dispenser, send \"ADD P<number>, <amount>\"");
  sim800LSerial.println("Since I have five dispensers, you can only choose P1-P5.");
  sim800LSerial.println("Similarly, I can only hold 10 pills at a time you can add up to 10 pills at a time.");
  sim800LSerial.println(" ");
  sim800LSerial.println("To know the schedules of each pill, just send \"SHOW SCHEDULES\".");
  sim800LSerial.println("To know how many pills and water there are as of the moment, just send \"SHOW STORAGE\".");
  sim800LSerial.print("To bring up this help menu again, just send \"SHOW HELP\".");

  updateSerial();
  sim800LSerial.write(26);
  delay(5000);
}

void help(){
  Serial.println("Sending help message to registered number.");
  sim800LSerial.println("AT+CMGF=1");
  updateSerial();
  sim800LSerial.println(destNumber);
  updateSerial();
  sim800LSerial.println("To SET the amount of pills in all dispensers, send \"SET ALL, <amount>\"");
  sim800LSerial.println("To SET the amount of pills in a specific dispenser, send \"SET P<number>, <amount>\"");
  sim800LSerial.println("To ADD an amount of pills in all dispensers, send \"ADD ALL, <amount>\"");
  sim800LSerial.println("To ADD an amount of pills in a specific dispenser, send \"ADD P<number>, <amount>\"");
  sim800LSerial.println("Since I have five dispensers, you can only choose P1-P5.");
  sim800LSerial.println("Similarly, I can only hold 10 pills at a time you can add up to 10 pills at a time.");
  sim800LSerial.println(" ");
  sim800LSerial.print("To bring up this help menu again, just send \"SHOW HELP\".");

  updateSerial();
  sim800LSerial.write(26);
}

void handleError(){
  sendMessage("ERROR!");
}

void sendMessage(String message){
  Serial.println("Sending message to registered number.");
  sim800LSerial.println("AT+CMGF=1");
  updateSerial();
  sim800LSerial.println(destNumber);
  updateSerial();
  sim800LSerial.print(message); //Your message
  updateSerial();
  sim800LSerial.write(26);
  delay(2000);
}

void scheduleTelemetry(){
  Serial.println("Sending schedule to registered number.");
  sim800LSerial.println("AT+CMGF=1");
  updateSerial();
  sim800LSerial.println(destNumber);
  updateSerial();
  sim800LSerial.println("Pill schedule:");
  sim800LSerial.println(" ");
  for(int i=0; i < 5; i++){
    String str = String("Pill dispenser ") + String(i+1) + String(": ");
    sim800LSerial.println(str);
    for(int j=0; j < MAX_NUM_SCHEDULES; j++){
      String str1 = String(j+1) + String(": ");
      sim800LSerial.print(str1);
      if(schedules[i][j].Hour == 25) {
        sim800LSerial.println("Schedule not set");
      } else {
        String hour = (schedules[i][j].Hour < 10) ? String("0") + String(schedules[i][j].Hour) : String(schedules[i][j].Hour);
        String minute = (schedules[i][j].Minute < 10) ? String("0") + String(schedules[i][j].Minute) : String(schedules[i][j].Minute);
        String str2 = hour + String(":") + minute;
        sim800LSerial.println(str2);
      }
    } if(i < 4) sim800LSerial.println(" ");
  }
  updateSerial();
  sim800LSerial.write(26);
}

void storageTelemetry(){
  Serial.println("Sending storage information to registered number.");
  sim800LSerial.println("AT+CMGF=1");
  updateSerial();
  sim800LSerial.println(destNumber);
  updateSerial();
  sim800LSerial.println("Pill storage:");
  sim800LSerial.println(" ");
  for(int i=0; i < 5; i++){
    String str = String("Pill dispenser ") + String(i+1) + String(": ");
    sim800LSerial.print(str);
    if(pillCount[i] == -1) sim800LSerial.println(String(pillCount[i] + 1));
    else sim800LSerial.println(String(pillCount[i]));
  }
  sim800LSerial.print("");
  updateSerial();
  sim800LSerial.write(26);
}

float tankLevelTelemetry(){
  float distance = tankSensor.measureDistanceCm();
  Serial.print(F("Tank Level: "));
  Serial.println(distance);
  return distance;
}

float cupTelemetry(){
  float distance = cupSensor.measureDistanceCm();
  Serial.print(F("Cup distance: "));
  Serial.println(distance);
  return distance;
}

void resetRelay(int pin){
  digitalWrite(pin, LOW);
}

void toggleRelay(int pin){
  digitalWrite(pin, !digitalRead(pin));
}

void blinkAndBuzz(){
  for(int i=0; i < BLINK_TIMES; i++){
    toggleRelay(LED_PIN);
    toggleRelay(BUZZER_PIN);
    delay(BLINK_DELAY);
    toggleRelay(LED_PIN);
    toggleRelay(BUZZER_PIN);
    delay(BLINK_DELAY);
  }
}

void setup() {
  // SET PINMODES
  pinMode(TANK_TRIG_PIN, OUTPUT);
  pinMode(TANK_ECHO_PIN, INPUT);
  pinMode(CUP_TRIG_PIN, OUTPUT);
  pinMode(CUP_ECHO_PIN, INPUT);

  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  // SET PINMODES


  Serial.begin(9600);
  sim800LSerial.begin(9600);
  rtc.begin(); // Initialize the rtc object
  tmElements_t rtcTime;
  String compileDate = __DATE__;
  String compileTime = __TIME__;
  rtcTime.Month = (uint8_t) monthToInt(compileDate.substring(0,3));
  rtcTime.Day = (uint8_t) compileDate.substring(4,6).toInt();
  rtcTime.Year = (uint8_t) compileDate.substring(7).toFloat() - 1970;
  rtcTime.Hour = (uint8_t) compileTime.substring(0,2).toInt();
  rtcTime.Minute = (uint8_t) compileTime.substring(3,6).toInt();
  rtcTime.Second = (uint8_t) compileTime.substring(6).toInt();


  time_t  tt = makeTime(rtcTime);
  rtc.set(tt);   // use the time_t value to ensure correct weekday is set
  setTime(tt);

  dispenser1.setMaxSpeed(1000.0);
  dispenser1.setAcceleration(100.0);
  dispenser1.setSpeed(200);
  
  dispenser2.setMaxSpeed(1000.0);
  dispenser2.setAcceleration(100.0);
  dispenser2.setSpeed(200);

  dispenser3.setMaxSpeed(1000.0);
  dispenser3.setAcceleration(100.0);
  dispenser3.setSpeed(200);

  dispenser4.setMaxSpeed(1000.0);
  dispenser4.setAcceleration(100.0);
  dispenser4.setSpeed(200);

  dispenser5.setMaxSpeed(1000.0);
  dispenser5.setAcceleration(100.0);
  dispenser5.setSpeed(200);


  // Reset number of pills
  for(int i = 0; i < 5; i++){
    pillCount[i] = 0;
  }

  // Reset snooze and dispensed flag
  snooze = 0;
  dispensedFlag = 0;

  // Reset schedules; 25 == invalid format == uninitialized
  for(int i=0; i < 5; i++){
    numSchedules[i] = 0;
    for(int j=0; j < MAX_NUM_SCHEDULES; j++) schedules[i][j].Hour = 25;
  }

  // Initialize GSM Module

  /////// UNCOMMENT IF GSM MODULE WILL BE USED FOR TESTING AND DEPLOYMENT

  while(!sim800LSerial.available()){
    sim800LSerial.println("AT");
    delay(1000); 
    Serial.println("Connecting...");
  } Serial.println("Connected!");  
  sim800LSerial.println("AT+CSQ");
  updateSerial();
  sim800LSerial.println("AT+CCID");
  updateSerial();
  sim800LSerial.println("AT+CREG?");
  updateSerial();
  sim800LSerial.println("AT+CNMI?");
  updateSerial();
  sim800LSerial.println("AT+CSCS=\"GSM\"");
  updateSerial();
  sim800LSerial.println("AT+CMGF=1");  //Set SMS to Text Mode 
  updateSerial();
  delay(1000);  
  sim800LSerial.println("AT+CNMI=1,2,0,0,0");  // Handle newly arrived messages(command name in text: new message indications to TE) 
  updateSerial();
  delay(1000);
  sim800LSerial.println("AT+CMGL=\"REC UNREAD\""); // Read Unread Messages
  updateSerial();

  /////// UNCOMMENT IF GSM MODULE WILL BE USED FOR TESTING AND DEPLOYMENT


  // HARDCODED SCHEDULES
  tmElements_t s;
  s.Hour = 16;
  s.Minute = 58;
  schedules[0][0] = s;
  s.Hour = 13;
  s.Minute = 24;
  schedules[0][1] = s;
  // HARDCODED SCHEDULES

  // // HARDCODED PILL COUNTS
  pillCount[0] = 5;
  pillCount[1] = 5;
  pillCount[2] = 5;
  pillCount[3] = 5;
  pillCount[4] = 5;
  // // HARDCODED PILL COUNTS


  firstBoot(); // Comment out this line to get rid of boot up message
  delay(10000);
  // blinkAndBuzz();
  // Put code here for testing like the ones below for quick functionality tests

  // for(int i=0; i < 5; i++){
  //   String str = String("Pill dispenser ") + String(i+1) + String(": ");
  //   Serial.println(str);
  //   for(int j=0; j < MAX_NUM_SCHEDULES; j++){
  //     String str1 = String(j+1) + String(": ");
  //     Serial.print(str1);
  //     if(schedules[i][j].Hour == 25) {
  //       Serial.println("Schedule not set");
  //     } else {
  //       String str2 = String(schedules[i][j].Hour) + String(":") + String(schedules[i][j].Minute);
  //       Serial.println(str2);
  //     }
  //   }
  // }

    // digitalWrite(WATER_PUMP_PIN, HIGH);
    // delay(pumpBuffer);//
    // digitalWrite(WATER_PUMP_PIN, LOW);
      // dispenser1.moveTo(endPoint);
      // dispenser2.moveTo(endPoint);
      // dispenser3.moveTo(endPoint);
      // dispenser4.moveTo(endPoint);
      // dispenser5.moveTo(endPoint);

}

void loop() { 
  // goto recv;
  // CHECK LOOP
  skipFlag = 0; // always reset skip flag
  // tankSkipFlag = 0;

  // Serial.println("Time:");
  // Serial.println(String(hour() + ":" + minute()));

  if(snooze == 1) goto dispense;

  // Check dispensers
  int empty[] = {0,0,0,0,0};
  for(int i = 0; i < 5; i++){
    // delay(3000);
    if(pillCount[i] == 0){
      empty[i] = 1;
      // String msg = String("Pill dispenser ") + String(i+1) + String(" is empty!");
      pillCount[i] = -1;
      // sendMessage(msg);
      // skipFlag = 1;
      blinkAndBuzz();
    } else if(pillCount[i] == -1){
      skipFlag = 1;
    }
  }

  if(skipFlag == 0){
    bool hasEmpty = false;
    String emptyDispensers;
    emptyDispensers.reserve(15);
    emptyDispensers = "";
    for(int i = 0; i < 5; i++){
      if(empty[i] == 1){
        hasEmpty = true;
        emptyDispensers.concat((emptyDispensers == "") ? String(i+1) : String(", ") + String(i+1));
        // emptyDispensers = emptyDispensers + String(i+1) + String(", ");
        Serial.println(emptyDispensers);
      } 
    }
    if(hasEmpty){
      if(emptyDispensers.length() > 1){
        String temp = emptyDispensers.substring(emptyDispensers.length() - 1);
        emptyDispensers = emptyDispensers.substring(0, emptyDispensers.length() - 1) + String("and ") + temp;
      }
      delay(3000);
      String msg = (emptyDispensers.length() == 1) ? String("Pill dispenser ") + emptyDispensers + String(" is empty!") : String("Pill dispensers ") + emptyDispensers + String(" are empty!");
      sendMessage(msg);
      skipFlag = 1;
    }
  }
  
  if(skipFlag == 1) goto recv;

  // Check water tank
  tank_level = tankLevelTelemetry();
  delay(1000);

  if(tank_level_previous != 0 && (int) tank_level >= (int) tank_level_previous){
    tank_level_previous = tank_level;
    goto recv;
  }
  if(tank_level >= tankLevelLow){
    tank_level_previous = tank_level;
    String msg = String("Tank level low. Please refill with clean water!");
    sendMessage(msg);
    blinkAndBuzz();
    goto recv;
  }

  if(dispensedFlag == 1) goto checkForRst;

  // CHECK SCHEDULES
  int toDispense[] = {0,0,0,0,0};
  for(int i=0; i < 5; i++){
    for(int j=0; j < MAX_NUM_SCHEDULES; j++){
      if(schedules[i][j].Hour == 25) continue;
      if(schedules[i][j].Hour == hour() && (minute() - schedules[i][j].Minute <= MAX_SNOOZES)){
        toDispense[i] = 1;
        snoozeTimer.Hour = hour();
        snoozeTimer.Minute = minute();
        String msg = String("Hello! Its now time to take ") + pillNames[i];
        sendMessage(msg);
        delay(5000);
      }
    }
  }

  for(int i=0; i < 5; i++){
    if (toDispense[i] == 1) goto dispense;
  } goto recv;

  dispense:
  // CHECK IF CUP IS READY
  cup_distance = cupTelemetry();
  if(cup_distance <= cupThreshold){
    delay(1000);
    // WAIT FOR 1s first before dispensing

    // DISPENSE WATER
    digitalWrite(WATER_PUMP_PIN, HIGH);
    delay(pumpBuffer);
    digitalWrite(WATER_PUMP_PIN, LOW);

    if(toDispense[0] == 1) {
      dispenser1.moveTo(endPoint);
      // dispenser1.step(stepsPerRevolution * REVOLUTIONS);
    } if(toDispense[1] == 1) {
      dispenser2.moveTo(endPoint);
      // dispenser2.step(stepsPerRevolution * REVOLUTIONS);
    } if(toDispense[2] == 1) {
      dispenser3.moveTo(endPoint);
      // dispenser3.step(stepsPerRevolution * REVOLUTIONS);
    } if(toDispense[3] == 1) {
      dispenser4.moveTo(endPoint);
      // dispenser4.step(stepsPerRevolution * REVOLUTIONS);
    }  if(toDispense[4] == 1) {
      dispenser5.moveTo(endPoint);
      // dispenser5.step(stepsPerRevolution * REVOLUTIONS);
    } 
    
    dispensedFlag = 1;
    goto resetSnooze;
  } else{
    if(snooze == 0){
      lastSnooze.Hour = hour();
      lastSnooze.Minute = minute();
      snooze = 1;
    } sendMessage("No cup detected! Please place cup below the water dispenser to dispense your pill/s");
  }


  if(minute() - snoozeTimer.Minute <= MAX_SNOOZES){
    if(minute() - lastSnooze.Minute > 0){
      snoozeTimer.Hour = hour();
      snoozeTimer.Minute = minute();
      blinkAndBuzz();
      // goto recv;
    }
  }

  resetSnooze:
  snooze = 0;

  checkForRst:
  if(snoozeTimer.Minute - minute() > MAX_SNOOZES){
    dispensedFlag = 0;
  } 



  // CHECK LOOP
  recv:
  if(sim800LSerial.available()){
    delay(100);

    // Serial Buffer
    while(sim800LSerial.available()){
      incomingByte = sim800LSerial.read();
      inputString += incomingByte; 
    }

      delay(10);      

      Serial.println(inputString);
      inputString.toUpperCase(); // Case insensitivity
    

      // COMMANDS:
      // SET
      // ADD
      // SCHEDULE SET
      // SCHEDULE REMOVE
      // SHOW SCHEDULE
      // SHOW STORAGE
      // SHOW HELP

      // SYNTAX:
      // SET destination, amount
      // ADD destination, amount
      // destination: ALL | P1 | P2 | P3 | P4 | P5
      // amount: 1 to (MAX_NUM_PILLS - current amount)

      // SCHEDULE SET destination, HH:MM
      // SCHEDULE REMOVE destination, index
      // destination: ALL | P1 | P2 | P3 | P4 | P5
      // HH: 00 to 24
      // MM: 00 to 59
      // index: 1 to (MAX_NUM_SCHEDULES - current number of schedules)

      inputString = inputString.substring(inputString.lastIndexOf('"') + 1);
      inputString.trim();

      if(inputString.indexOf("SHOW SCHEDULES") > -1){
        scheduleTelemetry();
        goto skip;
      } if(inputString.indexOf("SHOW STORAGE") > -1){ 
        storageTelemetry();
        goto skip;
      } if(inputString.indexOf("SHOW HELP") > -1){ 
        help();
        goto skip;
      }
      if(inputString.indexOf("SCHEDULE") > -1){ 
        if(inputString.indexOf("SET") > - 1){
          if(inputString.indexOf(",") > -1 && inputString.indexOf(":") > -1 ){
            String temp;
            temp = inputString;
            temp = temp.substring(temp.indexOf(",") + 1);
            temp.trim();
            int hh = temp.substring(0, temp.indexOf(":")).toInt();
            int mm = temp.substring(temp.indexOf(":") + 1).toInt();

            tmElements_t s;
            s.Hour = hh;
            s.Minute = mm;

            if(inputString.substring(12).indexOf("ALL") > -1){
              for(int i = 0; i < 5; i++){
                  if(numSchedules[i] == MAX_NUM_SCHEDULES) { 
                    handleError();
                    goto skip;
                  }
              }
              for(int i = 0; i < 5; i++){
                schedules[i][numSchedules[i]] = s;
                numSchedules[i]++;
              } sendMessage("Set schedule successful!");
            } 
            
            if(inputString.substring(13)[0] == 'P'){
              temp = inputString.substring(13)[1];
              // temp.remove(0);
              if(temp.toInt() - 1 > 5 || numSchedules[temp.toInt()] == MAX_NUM_SCHEDULES){
                handleError();
                goto skip;
              }

                schedules[temp.toInt() - 1][numSchedules[temp.toInt() - 1]] = s;
                numSchedules[temp.toInt() - 1]++;
                sendMessage("Set schedule successful!");
            }
            
            // else{
            //     temp = inputString;
            //     temp.remove(0);
            //     if(temp.toInt() - 1 > 5  || numSchedules[temp.toInt()] == MAX_NUM_SCHEDULES){
            //       handleError();
            //       goto skip;
            //     }
            //     schedules[temp.toInt() - 1][numSchedules[temp.toInt() - 1]] = s;
            //     numSchedules[temp.toInt() - 1]++;
            //     sendMessage("Set schedule successful!");
            //     goto skip;
            //   }
          } 
        } else if(inputString.indexOf("REMOVE") > - 1){
          if(inputString.indexOf(",") > -1){
            String temp;
            temp = inputString;
            temp = temp.substring(temp.indexOf(",") + 1);
            temp.trim();
            int index = temp.toInt();
            delay(2000);

            if(inputString.substring(16)[0] == 'P'){
              temp = inputString.substring(16)[1];
              // temp.remove(0);
              if(temp.toInt() - 1 > 5 || index > numSchedules[temp.toInt()-1]){
                handleError();
                goto skip;
              }
              schedules[temp.toInt() - 1][index- 1].Hour = 25;
              numSchedules[temp.toInt() - 1]--;

              sendMessage("Remove schedule successful!");
            }
          }
        } else handleError(); 
      } else if(inputString.indexOf("SET") > -1){
        if(inputString.indexOf(",") > -1){
          String temp;
          temp = inputString;
          temp = temp.substring(temp.indexOf(",") + 1);
          temp.trim();
          int amount = temp.toInt();
          if(amount > MAX_NUM_PILLS){
            handleError();
            goto skip;
          }

          if(inputString.substring(4).indexOf("ALL") > -1){
            Serial.print("Setting all to ");
            Serial.println(amount);
            for(int i = 0; i < 5; i++){
              pillCount[i] = amount;
            } 
            sendMessage("Set pill count successful!");
          } else{
            temp = inputString.substring(4,6);
            temp.trim();
            temp = temp.substring(1);
            if(temp.toInt() - 1 > 5  || amount > MAX_NUM_PILLS){
              handleError();
              goto skip;
            }
            pillCount[temp.toInt() - 1] = amount;
            for(int i = 0; i < 5; i++){
              Serial.println(pillCount[i]);
            } 
            Serial.println("INDEX");
            Serial.println(temp.toInt() -1);

            delay(2000);
            sendMessage("Set pill count successful!");
          }
        } else handleError();
      } else if(inputString.indexOf("ADD") > - 1){
          if(inputString.indexOf(",") > -1){
            String temp;
            temp = inputString;
            temp = temp.substring(temp.indexOf(",") + 1);
            temp.trim();
            int amount = temp.toInt();
            if(amount > MAX_NUM_PILLS){
              handleError();
              goto skip;
            }

            if(inputString.substring(4).indexOf("ALL") > -1){
              for(int i = 0; i < 5; i++){
                if(amount > MAX_NUM_PILLS - pillCount[i]){
                  handleError();
                  goto skip;
                }
              }

              for(int i = 0; i < 5; i++){
                if(pillCount[i] == -1) pillCount[i]++;
                pillCount[i] += amount;
              } 
              sendMessage("Add pill/s successful!");
            } else{
              temp = inputString.substring(4,6);
              temp.trim();
              temp = temp.substring(1);
              if(temp.toInt() - 1 > 5  || amount > MAX_NUM_PILLS){
                handleError();
                goto skip;
              }
              if(pillCount[temp.toInt() - 1] == -1) pillCount[temp.toInt() - 1]++;
              pillCount[temp.toInt() - 1] += amount;
              sendMessage("Add pill/s successful!");
            }
          } else handleError();
      }


      skip:
      delay(50);

      //Delete Messages & Save Memory
      if (inputString.indexOf("OK") == -1){
        sim800LSerial.println("AT+CMGDA=\"DEL ALL\"");

        delay(1000);
      }

      inputString = "";
      // Serial.println("Pill counts:");
      // for(int i = 0; i < 5; i++){
      //   Serial.print("Pill dispenser ");
      //   Serial.print(i);
      //   Serial.print(": ");
      //   Serial.println(pillCount[i]);
      // } 
  }
  
  while (dispenser1.distanceToGo() != 0){
    dispenser1.run();
  }if(dispenser1.distanceToGo() == 0){
    dispenser1.setCurrentPosition(0);
  }
  while (dispenser2.distanceToGo() != 0){
    dispenser2.run();
  }if(dispenser1.distanceToGo() == 0){
    dispenser2.setCurrentPosition(0);
  }
  while (dispenser3.distanceToGo() != 0){
    dispenser3.run();
  }if(dispenser1.distanceToGo() == 0){
    dispenser3.setCurrentPosition(0);
  }
  while (dispenser4.distanceToGo() != 0){
    dispenser4.run();
  }if(dispenser1.distanceToGo() == 0){
    dispenser4.setCurrentPosition(0);
  }
  while (dispenser5.distanceToGo() != 0){
    dispenser5.run();
  }if(dispenser1.distanceToGo() == 0){
    dispenser5.setCurrentPosition(0);
  }
//  delay(1000); 
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    sim800LSerial.write(Serial.read());
  }
  while(sim800LSerial.available()) 
  {
    Serial.write(sim800LSerial.read());
  }
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(month());
    Serial.print(' ');
    Serial.print(year());
    Serial.println();
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(':');
    if(digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

int monthToInt(String month){
  // Serial.println(month);
  if(month == "Jan") return 1;
  if(month == "Feb") return 2;
  if(month == "Mar") return 3;
  if(month == "Apr") return 4;
  if(month == "May") return 5;
  if(month == "Jun") return 6;
  if(month == "Jul") return 7;
  if(month == "Aug") return 8;
  if(month == "Sep") return 9;
  if(month == "Oct") return 10;
  if(month == "Nov") return 11;
  if(month == "Dec") return 12;
}