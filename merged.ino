#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <MechaQMC5883.h>
#include <LiquidCrystal.h>



MPU6050 mpu;
MechaQMC5883 qmc;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x70);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x71);
Adafruit_MLX90614 mlx3 = Adafruit_MLX90614(0x72);
Adafruit_MLX90614 mlx4 = Adafruit_MLX90614(0x73);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2)
bool b_x;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

int previousDegree;

float accPitch = 0;
float accRoll = 0;
float kalPitch = 0;
float kalRoll = 0;

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

void loop(){
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;
 
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);


  double T1 = mlx1.readObjectTempC();
  double LT = mlx2.readObjectTempC();
  double T3 = mlx3.readObjectTempC();
  double PT = mlx4.readObjectTempC();

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

  
  b_x = !b_x;
  delay(10);
  
}
