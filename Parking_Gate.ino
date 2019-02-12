#include "Arduino.h"
#include <Servo.h>

#define GREEN_LED_PIN 2
#define RED_LED_PIN 5
#define BUZZER_PIN 4

Servo myServo;
int servoPos = 0;

class Ultrasonic
{
	public:
		Ultrasonic(int pin);
        void DistanceMeasure(void);
		long microsecondsToCentimeters(void);
		long microsecondsToInches(void);
	private:
		int ULTRASONIC_RANGER_PIN;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
                long duration;// the Pulse time received;
};

Ultrasonic::Ultrasonic(int pin)
{
	ULTRASONIC_RANGER_PIN = pin;
}

/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
    pinMode(ULTRASONIC_RANGER_PIN, OUTPUT);
	digitalWrite(ULTRASONIC_RANGER_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(ULTRASONIC_RANGER_PIN, HIGH);
	delayMicroseconds(5);
	digitalWrite(ULTRASONIC_RANGER_PIN,LOW);
	pinMode(ULTRASONIC_RANGER_PIN,INPUT);
	duration = pulseIn(ULTRASONIC_RANGER_PIN,HIGH);
}

/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
	return duration/29/2;	
}

/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
	return duration/74/2;	
}
 
Ultrasonic ultrasonic(7);

void setup()
{
	Serial.begin(9600);
        pinMode(GREEN_LED_PIN, OUTPUT);
        pinMode(RED_LED_PIN, OUTPUT);
        pinMode(BUZZER_PIN, OUTPUT);
        myServo.attach(6);
 }

void loop()
{
	long RangeInInches;
	long RangeInCentimeters;
	ultrasonic.DistanceMeasure();// get the current signal time;
        RangeInInches = ultrasonic.microsecondsToInches();//convert the time to inches;
	RangeInCentimeters = ultrasonic.microsecondsToCentimeters();//convert the time to centimeters
	Serial.println("The distance of the vehicle is: ");
	// Serial.print(RangeInInches);//0~157 inches
	// Serial.println(" inch");
	Serial.print(RangeInCentimeters);//0~400cm
	Serial.println(" cm");
	delay(1000);


        digitalWrite(GREEN_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
        digitalWrite(RED_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)

        if (RangeInCentimeters <= 10){
        
            for(servoPos=0; servoPos <=90; servoPos++){
            myServo.write(servoPos);
            delay(15); // delay of Servo, means how fast the gate will open
            }
            
            digitalWrite(GREEN_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            digitalWrite(RED_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
            
            Serial.println("Parking Gate is open");

            for (int i =0; i<10;i++){
            tone(BUZZER_PIN, 1000); // Send 1KHz sound signal...
            delay(500);        // ...for 1 sec
            noTone(BUZZER_PIN);     // Stop sound...
            delay(500);        // ...for 1sec
            }
           // delay(5000); // the gate will be opened for 5 second 

            digitalWrite(GREEN_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
            digitalWrite(RED_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            
            for(servoPos = 90; servoPos >=0; servoPos--){
            myServo.write(servoPos);
            delay(30); // delay of Servo, means how slow the gate will close
            }
        }

        Serial.println("Parking Gate is closed");

}
