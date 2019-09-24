//#include <SPI.h>
//#include <SD.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <LiquidCrystal.h>
//#define BUTTON_PIN 7


LiquidCrystal lcd(9, 8, 5, 6, 3, 2);
 
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x70);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x71);
Adafruit_MLX90614 mlx3 = Adafruit_MLX90614(0x72);
Adafruit_MLX90614 mlx4 = Adafruit_MLX90614(0x73);

//const int chipSelect = 4;

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect)) {
  //  Serial.println("Card failed, or not present");
    // don't do anything more:
   // while (1);
 // }
 // Serial.println("card initialized.");  
  
  mlx1.begin();  
  mlx2.begin(); 
  mlx3.begin(); 
  mlx4.begin(); 

  lcd.begin(16, 2);

  //pinMode(BUTTON_PIN, INPUT);
 // digitalWrite(BUTTON_PIN, HIGH); 
}

//boolean handle_button()
//{
//  int button_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
 // return button_pressed;
//}
 
void loop()
{  
  //String dataString = "";
   // File mySensorData = SD.open("datalog.txt", FILE_WRITE);
    double T1 = mlx1.readObjectTempC();
    double T2 = mlx2.readObjectTempC();
    double T3 = mlx3.readObjectTempC();
    double T4 = mlx4.readObjectTempC();

 // dataString += "T1="+String(T1);
 // dataString += "T2="+String(T2);
 // dataString += "T3="+String(T3);
 // dataString += "T4="+String(T4);
  
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T1="); lcd.print(T1);
  
    lcd.setCursor(9, 0);
    lcd.print("T2="); lcd.print(T2);
  
    lcd.setCursor(0, 1);
    lcd.print("T3="); lcd.print(T3);
  
    lcd.setCursor(9, 1);
    lcd.print("T4="); lcd.print(T4);
 
 // if (mySensorData) {
 //   Serial.println("opening txt file!");

     
  //  mySensorData.println(dataString);
 //   mySensorData.close();
    
  //}else {
 //  Serial.println("error opening txt file!");
 // }
  delay(500);
  
  // boolean button_pressed = handle_button();
  // if(button_pressed){
  //  mySensorData.close();
  //  Serial.println("GUZIOR");
   // delay(10000);
 //}
}
