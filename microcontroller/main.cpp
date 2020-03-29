#include <Arduino.h>
#include <LiquidCrystal.h>

#include "../common/Inc/SerialCommunication.h"

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }

int main(void)
{
    init();

    initVariant();

#if defined(USBCON)
    USBDevice.attach();
#endif
    
    setup();
    
    for (;;) {
        loop();
        if (serialEventRun) serialEventRun();
    }
        
    return 0;
}

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 

    
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

SerialFeedback feedback;
SerialFeedback newFeedback;

SerialCommand command;

#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication


void setup()
{
  Serial.begin(38400);
  //HoverSerial.begin(38400);

  feedback.leftMotorSettings = defaultMotorSettings();
  feedback.rightMotorSettings = defaultMotorSettings();

  command.overrideMotorInput = (uint16_t)false;
  command.leftMotorInput.pwm = 0;
  command.rightMotorInput.pwm = 0;

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("");
  lcd.setCursor(0,1);

}

void receiveFeedback(void)
{
    // Check for new data availability in the Serial buffer
    if (Serial.available()) {
            incomingByte 	  = Serial.read();		                              // Read the incoming byte
            bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	  // Construct the start frame
    }
    else {
            return;
    }

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
            p 		= (byte *)&newFeedback;
            *p++  = incomingBytePrev;
            *p++ 	= incomingByte;
            idx 	= 2;

    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {	// Save the new received data
            *p++ 	= incomingByte;
            idx++;
    }

    //Serial.println(sizeof(SerialFeedback));

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
            uint16_t checksum;
            checksum = (uint16_t)calculateChecksumFeedback(newFeedback);

            // Check validity of the new data
            if (newFeedback.start == START_FRAME && checksum == newFeedback.checksum) {
                    // Copy the new data
                    memcpy(&feedback, &newFeedback, sizeof(SerialFeedback));

                    // Print data to built-in Serial
                    //Serial.print("cmd1: ");   Serial.print(feedback.cmd1);
                    //Serial.print(" cmd2: ");  Serial.print(feedback.cmd2);
                    //Serial.print("adc1: ");   Serial.print(feedback.adc1);
                    //Serial.print(" adc2: ");  Serial.print(feedback.adc2);
                    //Serial.print(" speedR_meas: ");  Serial.print(feedback.speedR_meas);
                    //Serial.print(" speedL_meas: ");  Serial.print(feedback.speedL_meas);
                    //Serial.print(" batVoltage: ");  Serial.print(feedback.batVoltage);
                    //Serial.print(" boardTemp: ");  Serial.print(feedback.boardTemp);
                    //Serial.print(" 7: ");  Serial.print(feedback.cmdLed);

                    //lcd.setCursor(0,0);
                    //lcd.print("0: "); lcd.print(feedback.leftMotorInput.pwm);
                    //lcd.setCursor(0,1);
                    //lcd.print("1: "); lcd.print(feedback.leftMotorInput.pwm);

                    lcd.setCursor(0,0);
                    lcd.print("pwm1: "); lcd.print(feedback.leftMotorInput.pwm);

                    lcd.setCursor(0,1);
                    lcd.print("pwm2: "); lcd.print(feedback.rightMotorInput.pwm);
            } else {
              //Serial.println("Non-valid data skipped");
            }
            idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev 	= incomingByte;
    //else
    //{
    //lcd.clear();
    //lcd.setCursor(0,1);
    //lcd.print("Board offline");
    //}

    //delay(500);


}

void transmitCommand(void)
{
    // Create command
    command.start = START_FRAME;

    command.checksum = calculateChecksumCommand(command);

    // Write to Serial
    Serial.write((uint8_t *) &command, sizeof(command));
}

#define TIME_SEND           100


unsigned long iTimeSend = 0;

void loop()
{

    unsigned long timeNow = millis();

    receiveFeedback();


    // TEST CODE

    command.leftMotorInput = feedback.leftMotorInput;
    command.rightMotorInput = feedback.rightMotorInput;

    command.leftMotorSettings = feedback.leftMotorSettings;
    command.rightMotorSettings = feedback.rightMotorSettings;


    int x;

    x = analogRead(0);
    if (x < 60)
    {
      command.overrideMotorInput = (uint16_t)true;
      command.leftMotorInput.pwm = 100;
      command.rightMotorInput.pwm = -100;
    }
    else if (x < 200)
    {
      //lcd.print ("Up    ");
    }
    else if (x < 400)
    {
      //lcd.print ("Down  ");
    }
    else if (x < 600)
    {
      //lcd.print ("Left  ");
    }
    else if (x < 800)
    {
     //lcd.print ("Select");
    }

    else
    {
        command.overrideMotorInput = false;
    }

    // TEST CODE END


    // Send commands
    if (iTimeSend > timeNow) return;

    iTimeSend = timeNow + TIME_SEND;

    transmitCommand();
}


