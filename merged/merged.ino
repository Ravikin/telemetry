// @TODO trzeba jeszcze sprawdzic ktory mlx z przodu jest na ktorej stronie


#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <MechaQMC5883.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// inicjalizacja zmiennych
MPU6050 mpu;
MechaQMC5883 qmc;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x70);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x71);
Adafruit_MLX90614 mlx3 = Adafruit_MLX90614(0x72);
Adafruit_MLX90614 mlx4 = Adafruit_MLX90614(0x73);
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
bool b_x = true;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

int previousDegree;

float accPitch = 0;
float accRoll = 0;
float kalPitch = 0;
float kalRoll = 0;

#define GPSECHO true  

// setup boarda razem z bledem nt. gyro
// to trzeba bedzie poprawic xd @TODO
// tak zeby mowilo ze jest problem ale sie odpalilo...no albo zeby chociaz mowilo o problemie
void setup(){
  Serial.begin(115200);
  delay(5000);
  Serial.println("Setup");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("no mpu");
    delay(500);
  }
  Serial.println("mpu end");
  mpu.calibrateGyro();
  mlx1.begin();
  mlx2.begin();
  mlx3.begin();
  mlx4.begin();
  qmc.init();
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);



  lcd.begin(16,2);
}
uint32_t timer = millis();

// i lecimy z koksem
void loop(){
// zbieramy dane z gyro mpu
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();
// obliczamy wychylenia i "przyspieszenia"
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;
 // normalizacja kalmana
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

// zbieramy temperaturki
  double PP = mlx1.readObjectTempC();
  double LT = mlx2.readObjectTempC();
  double LP = mlx3.readObjectTempC();
  double PT = mlx4.readObjectTempC();

// kierunek z magnetometru
  int x,y,z;
  qmc.read(&x,&y,&z);

  // Obliczenie kierunku (rad)
  float heading = atan2(y, x);
 
  // Ustawienie kata deklinacji dla Bytomia 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (6.0 + (2.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
 
  // Korekta katow
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
 
  // Zamiana radianow na stopnie
  float headingDegrees = heading * 180/M_PI;

  // GPS STUFF
    char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  
// wypisujemy dane na serial (cos po kablu)
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.print(GPS.year, DEC);
  Serial.print(";");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.print(GPS.milliseconds);
  Serial.print(";");
  if (GPS.fix) {
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(";");
    Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
    Serial.print(";");
    Serial.print((GPS.speed)*1.8);
  }
  Serial.print(accPitch);
  Serial.print(";");
  Serial.print(accRoll);
  Serial.print(";");
  Serial.print(kalPitch);
  Serial.print(";");
  Serial.print(kalRoll);
  Serial.print(";");
  Serial.print(LP);
  Serial.print(";");
  Serial.print(LT);
  Serial.print(";");
  Serial.print(PP);
  Serial.print(";");
  Serial.print(PT);
  Serial.print(";");
  Serial.println(headingDegrees);


// wyswietlamy dane na lcdku 16 znakow na 2 wiersze
// jezeli temperatura >=90 to niech blinka dana nazwa pozycji
// trzeba dodac ewentualny setup gdzie wyklikujesz graniczną temperature guziczkami @TODO
  lcd.setCursor(0,0);
  if (LP >= 90)
  {
		lcd.print("!!:"); lcd.print(LP);
  } else
  {
	lcd.print("LP:"); lcd.print(LP);
  }

  lcd.setCursor(9,0);
  if (PP >= 90) {
	  lcd.print("!!:"); lcd.print(PP);
  } else {
	  lcd.print("PP:"); lcd.print(PP);
  }
  
  lcd.setCursor(0,1);
  if (LT >= 90) {
	    lcd.print("!!:"); lcd.print(LT);
  } else {
	lcd.print("LT:"); lcd.print(LT);
  }

  lcd.setCursor(9,1);
  if (PT >= 90) {
	    lcd.print("!!:"); lcd.print(PT);
  } else {
	lcd.print("PT:"); lcd.print(PT);
  }

// zamieniamy blinkowego boola na przeciwny - mam nadzieje ze dziala xd
// btw nie wymyslilem lepszego sposobu...ten byl pierwszy XD
// @TODO? na pewno lepsiejszy wyswietlacz <3
  b_x = !b_x;
  // ewentualnie delaya mozna wywalic - to po to zeby.....w sumie nie pamietam. chyba po to zeby malinka dawala rade wszystko zapisywac? na pewno jak jest za duzy delay to ktorys z elementow szalal :/ 
  delay(500);
  
}
