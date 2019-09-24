// @TODO trzeba jeszcze sprawdzic ktory mlx z przodu jest na ktorej stronie


#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <MechaQMC5883.h>
#include <LiquidCrystal.h>


// inicjalizacja zmiennych
MPU6050 mpu;
MechaQMC5883 qmc;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x70);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x71);
Adafruit_MLX90614 mlx3 = Adafruit_MLX90614(0x72);
Adafruit_MLX90614 mlx4 = Adafruit_MLX90614(0x73);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2)
bool b_x = True;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

int previousDegree;

float accPitch = 0;
float accRoll = 0;
float kalPitch = 0;
float kalRoll = 0;

// setup boarda razem z bledem nt. gyro
// to trzeba bedzie poprawic xd @TODO
// tak zeby mowilo ze jest problem ale sie odpalilo...no albo zeby chociaz mowilo o problemie
void setup(){
  Serial.begin(115200);
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

  lcd.begin(16,2);
}

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
  double T1 = mlx1.readObjectTempC();
  double LT = mlx2.readObjectTempC();
  double T3 = mlx3.readObjectTempC();
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
  
// wypisujemy dane na serial (cos po kablu)
  Serial.print(accPitch);
  Serial.print(";");
  Serial.print(accRoll);
  Serial.print(";");
  Serial.print(kalPitch);
  Serial.print(";");
  Serial.print(kalRoll);
  Serial.print(";");
  Serial.print(T1);
  Serial.print(";");
  Serial.print(LT);
  Serial.print(";");
  Serial.print(T3);
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
	if (b_x)
	{
		lcd.print("  :"); lcd.print(T1);
	} else
	{
		lcd.print("LP:"); lcd.print(T1);
	}
  } else
  {
	lcd.print("LP:"); lcd.print(T1);
  }

  lcd.setCursor(9,0);
  if (PP >= 90) {
	if (b_x) {
		lcd.print("  :"); lcd.print(T3);
	} else {
		lcd.print("PP:"); lcd.print(T3);
	}
  } else {
	lcd.print("PP:"); lcd.print(T3);
  }
  
  lcd.setCursor(0,1);
  if (LT >= 90) {
	if (b_x) {
		lcd.print("  :"); lcd.print(LT);
	} else {
		lcd.print("LT:"); lcd.print(LT);
	}
  } else {
	lcd.print("LT:"); lcd.print(LT);
  }

  lcd.setCursor(9,1);
  if (PT >= 90) {
	if (b_x) {
		lcd.print("  :"); lcd.print(PT);
	} else {
		lcd.print("PT:"); lcd.print(PT);
	}
  } else {
	lcd.print("PT:"); lcd.print(PT);
  }

// zamieniamy blinkowego boola na przeciwny - mam nadzieje ze dziala xd
// btw nie wymyslilem lepszego sposobu...ten byl pierwszy XD
// @TODO? na pewno lepsiejszy wyswietlacz <3
  b_x = !b_x;
  // ewentualnie delaya mozna wywalic - to po to zeby.....w sumie nie pamietam. chyba po to zeby malinka dawala rade wszystko zapisywac? na pewno jak jest za duzy delay to ktorys z elementow szalal :/ 
  delay(10);
  
}
