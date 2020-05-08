 ////Breath Analyzer using ArduinoMega 2560
///
///

#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
/*
Devices Connected
1. Blue SCreen LCD
2. SIM900 GSM Module
3. Red LED
4. 3Vdc Buzzer
5. NEO GPS Module
6. MQ3 Sensor
*/
////////////////////////////PINS DESCRIPTION///////////////////////////////////////////////////
const int rs = 22;                    //Pins to LCD
const int en = 23;
const int d4 = 24;
const int d5 = 25;
const int d6 = 26;
const int d7 = 27;
const int SIM900_PWR_KEY = 28;
const int OFF_BoardLED = 12;          //PB12  OFF_BoardLED
const int BuzzerPin = 13;             //PB13  BuzzerPin
const int SIM900_RX = 14;
const int SIM900_TX = 15;
const int NEOGPS_RX = 16;
const int NEOGPS_TX = 17;
const int SensorPin = A15;              //the AOUT pin of MQ-3
////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////OBJECT CREATION////////////////////////////////////////////////////
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 
SoftwareSerial SIM900(SIM900_TX, SIM900_RX);  //Connect to Arduino(RX, TX)
SoftwareSerial NEOGPS(NEOGPS_TX, NEOGPS_RX);
TinyGPS gps; // create gps object
//////////////////////////////////////////////////////////////////////////////////////////////


////////////////////VARIABLES DECLARATION & INITIALIZATION////////////////////////////////////// 
float sensorValue=0.0;
float voltage = 5.0;
float multiplier = 0.21;
int ADC_bit_Value = 1024; //10-Bit ADC in Arduino implies mapping input voltages between 0 and 5 volts into integer values between 0 and 1023.
float BAC; //Blood Alcohol Content (BAC)= Breadth (mg/L) * multiplier  
float country_BAC_Limit = 0.05;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);//sets the baud rate for serial screen
  SIM900.begin(19200); //sets the baud rate for the GSM Module
  NEOGPS.begin(9600); //sets the baud rate for the GPS Module
  lcd.begin(16, 2);////Initialize the LCD => We are using a 16*2 LCD
  delay(20000); //give room for gsm and gps to log into network and satellite respectively
  pinMode(SensorPin, INPUT);
  pinMode(OFF_BoardLED, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);

  digitalWrite(SIM900_PWR_KEY, HIGH);
  delay(1000);
  digitalWrite(SIM900_PWR_KEY, LOW);
  delay(5000);
  
  BannerMessages("Welcome to Tony Breath Analyzer", "Welcome to Tony", "Breath Analyzer", 4000);
  BannerMessages("Detecting Modules...", "    Detecting   ", "   Modules...   ", 3000);
  BannerMessages("GPS Module Detected", "   GPS Module   ", "    Detected    ", 1500);
  BannerMessages("MQ3 Module Detected", "   MQ3 Module   ", "    Detected    ", 1500);
  BannerMessages("GSM Module Detected", "   GSM Module   ", "    Detected    ", 1500);
  BannerMessages("CAMERA Module Detected", "  Camera Module ", "    Detected    ", 1500);

  Serial.println("Please Exhale");    //Print on Serial
  lcd.setCursor(0, 0); //At first row first column 
  lcd.print("Please Exhale");         //Print this on LCD
  delay(2000); //wait for two secounds 
  lcd.clear(); //Clear the screen

  attachInterrupt(digitalPinToInterrupt(interruptPin), takePix, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), NoPix, CHANGE);
}


void loop()      
      {
      for (int i = 0 ; i < 10 ; i++)                          //take 10 readings from analog input
          {
           sensorValue +=  analogRead(SensorPin);
           delay(10);   
      }
      sensorValue /= 10.0;                                    //find the average of the 10 read readings
      BAC = (sensorValue/ADC_bit_Value)*multiplier; //calculate BAC using datasheet formula
      //BAC = sensorValue*(voltage/ADC_bit_Value)*multiplier;   //from circuitdigest
      
      if (BAC < country_BAC_Limit)                           //if the calculated BAC is less than country limit (as specified in country_BAC_Limit above)
            {
            //digitalWrite(ON_BoardLED, LOW);
            digitalWrite(OFF_BoardLED, LOW);
            digitalWrite(BuzzerPin, LOW);
            Serial.print("BAC Acceptable @");
            Serial.println(BAC);//prints the alcohol value
            Serial.println("");
            
            lcd.setCursor(0, 0); //At first row first column 
            lcd.print("BAC Acceptable @");         //Print this on LCD
            lcd.setCursor(0, 1); //At first second first column 
            lcd.print(BAC);         //Print this on LCD
            lcd.setCursor(5, 1); //At first second sixth column 
            lcd.print(" mg/L");         //Print this on LCD
            delay(2000); //wait for two secounds 
            lcd.clear(); //Clear the screen  
      }
      else {
            Serial.print("BAC Beyond Limit @: ");
            Serial.println(BAC);//prints the alcohol value
            Serial.println("");

            lcd.setCursor(0, 0); //At first row first column 
            lcd.print("BAC Beyond Limit");         //Print this on LCD
            lcd.setCursor(0, 1); //At first second first column 
            lcd.print(BAC);         //Print this on LCD
            lcd.setCursor(5, 1); //At first second sixth column 
            lcd.print(" mg/L");         //Print this on LCD
            for (int j = 0 ; j < 3 ; j++)
                {
                 //digitalWrite(ON_BoardLED, HIGH);
                 digitalWrite(OFF_BoardLED, HIGH);
                 digitalWrite(BuzzerPin, HIGH);
                 delay(500); 
                 digitalWrite(BuzzerPin, LOW);
                 delay(500);
                 if (j == 3)
                      {
                      //digitalWrite(ON_BoardLED, LOW);
                      digitalWrite(OFF_BoardLED, LOW);
                  }
            }
            if (NEOGPS.available())
              { // check for gps data
              if(gps.encode(NEOGPS.read()))// encode gps data
                { 
                gps.f_get_position(&lat,&lon); // get latitude and longitude

                Serial.println("GPS SIGNAL");
                Serial.println("==========");
                Serial.print("Longitude: ");
                Serial.println(lon, 4);
                Serial.print("Latitude: ");
                Serial.println(lat, 4);
            
                lcd.clear(); //Clear the screen
                lcd.setCursor(0, 0); //At first row first column 
                lcd.print("GPS SIGNAL");
                lcd.setCursor(0, 0); //At first row first column 
                lcd.print(lon,4);         //Print this on LCD
                lcd.setCursor(6, 0);
                lcd.print(", N");
                lcd.setCursor(0, 1); //At first row first column 
                lcd.print(lat,4);         //Print this on LCD
                lcd.setCursor(6, 1);
                lcd.print(", E");
                delay(1500); //wait for two secounds 
                lcd.clear(); //Clear the screen
                Send_SMS_Message(lon, lat);   // //The GSM Module Send SMS Message
              }
              else{
                  Serial.println("GPS READ BUT CANNOT BE ENCODED");
                  lcd.setCursor(0, 0);                                          //At first row first column 
                  lcd.print("GPS READ BUT");                                    //Print this on LCD
                  lcd.setCursor(0, 1);                                          //At SECOND row first column 
                  lcd.print("CANNOT BE ENCODE");                                //Print this on LCD
                  delay(2000);                                                  //wait for two secounds 
                  lcd.clear();                                                  //Clear the screen 
              }
           }
			Serial.println("GPS SIGNAL UNAVAILABLE");		
			lcd.clear(); //Clear the screen
			lcd.setCursor(0, 0); //At first row first column 
			lcd.print("GPS UNAVAILABLE");
      }
    Serial.println("Please Exhale into Sensor...");
    lcd.setCursor(0, 0); //At first row first column 
    lcd.print("Please Exhale");         //Print this on LCD
    delay(2000); //wait for two secounds 
    lcd.clear(); //Clear the screen  
}


void BannerMessages(char x[16], char y[16], char z[16], int d)
    {
    Serial.println(x);    //Print first arg on serial
    lcd.setCursor(0, 0); //At first row first column 
    lcd.print(y);         //Print the second arg on LCD
    lcd.setCursor(0, 1); //At first column second row 
    lcd.print(z);         //Print the second arg on LCD
    delay(d); //wait for d secounds 
    lcd.clear(); //Clear the screen
  }

void Send_SMS_Message(float longi, float lati)
{
  SIM900.println("AT+CMGF=1\r");    //Sets the GSM Module in Text Mode
  delay(100);  // Delay of 100 milli seconds
  SIM900.println("AT+CMGS=\"+2348035867979\"\r"); // Recipient mobile number
  delay(100);
  SIM900.print("Drunk found @ : ");    //E.g. Drunk found @ : 3.962N, 3.845E
  SIM900.print(longi, 4);
  SIM900.print("N, ");
  SIM900.print(lati, 4);
  SIM900.println("E");
  delay(100);
  SIM900.println((char)26);// ASCII code of CTRL+Z
  delay(100);
  SIM900.println();
  // Give module time to send SMS
  delay(5000);
  Serial.println("SMS Sent");
  lcd.setCursor(0, 0);                                      //At first row first column 
  lcd.print("SMS Sent");                                    //Print this on LCD
  delay(2000);                                              //wait for two secounds 
  lcd.clear(); 
}
