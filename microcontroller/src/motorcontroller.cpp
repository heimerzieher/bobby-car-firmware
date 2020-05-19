#include "motorcontroller.h"

MotorController::MotorController(HardwareSerial& serialInterface) : serial{serialInterface}
{
    this->feedback.leftMotorSettings = defaultMotorSettings();
    this->feedback.rightMotorSettings = defaultMotorSettings();

    this->command.overrideMotorInput = (uint16_t)false;
    this->command.leftMotorInput.pwm = 0;
    this->command.rightMotorInput.pwm = 0;
}

void MotorController::receiveFeedback(void)
{
    // Check for new data availability in the Serial buffer
    if (this->serial.available()) {
            incomingByte 	  = this->serial.read();		                              // Read the incoming byte
            bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	  // Construct the start frame
            this->isOnline = true;
    }
    else {
            this->serialTimeoutCounter++;
            if(this->serialTimeoutCounter > SERIAL_TIMEOUT_COUNTS)
                this->isOnline = false;
            return;
    }

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
            p 		= (byte *)&this->_feedback;
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
            checksum = (uint16_t)calculateChecksumFeedback(this->_feedback);

            // Check validity of the new data
            if (this->_feedback.start == START_FRAME && checksum == this->_feedback.checksum) {
                    // Copy the new data
                    memcpy(&feedback, &this->_feedback, sizeof(SerialFeedback));

                    this->serialTimeoutCounter = 0;
                    this->isOnline = true;
                    // Print data to built-in Serial
                    //Serial.print("cmd1: ");   Serial.println(feedback.cmd1);
                    //Serial.print(" cmd2: ");  Serial.println(feedback.cmd2);
                    //Serial.print("adc1: ");   Serial.println(feedback.adc1);
                    //Serial.print(" adc2: ");  Serial.println(feedback.adc2);
                    //Serial.print(" speedR_meas: ");  Serial.print(feedback.speedR_meas);
                    //Serial.print(" speedL_meas: ");  Serial.print(feedback.speedL_meas);
                    //Serial.print(" batVoltage: ");  Serial.print(feedback.batVoltage);
                    //Serial.print(" boardTemp: ");  Serial.print(feedback.boardTemp);
                    //Serial.print(" 7: ");  Serial.print(feedback.cmdLed);

            } else {
              //Serial.println("Non-valid data skipped");
            }
            idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev 	= incomingByte;

}


void MotorController::transmitCommand(void)
{
    // Create command
    this->command.start = START_FRAME;

    this->command.checksum = calculateChecksumCommand(this->command);

    // Write to Serial
    this->serial.write((uint8_t *) &this->command, sizeof(this->command));
}
