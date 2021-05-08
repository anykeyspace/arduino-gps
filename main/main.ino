#include <SoftwareSerial.h>
//начало вставки кода №1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//конец вставки кода №1
#include <TinyGPS.h>

#include <math.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial ss(4, 3);

struct anshlagPoint
{
  float lat;
  float lon;
};

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);
//начало вставки кода №2
LiquidCrystal_I2C lcd(0x27, 20, 4);
//конец вставки кода №2
void setup()
{
  Serial.begin(9600);
  
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum   Distance  ");
  Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail       (m)");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------------------");

  ss.begin(9600);
  
 //начало вставки кода №3
  lcd.begin();
lcd.backlight();
//конец вставки кода №3
}

void loop()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
  
  // Проверяемые точки. Широта и долгота в градусах. Порядок не имеет значения
  int pointsNumber = 4; // количество точек
  struct anshlagPoint points[pointsNumber];
  points[0] = {53.190194, 45.064625};
  points[1] = {53.190194, 45.064625};
  points[2] = {53.190194, 45.064625};
  points[3] = {53.190194, 45.064625};

  // Радиус зоны обнаружения в метрах
  float distance = 10;

  //Пин, к которому подключен пьезодинамик.
  int piezoPin = 2;

  //Проверка
  int result = 0;
  for (int i = 0; i < pointsNumber; ++i) {
    if (magic_method(flat, flon, points[i].lat, points[i].lon, distance)) {
      result = 1;
      break;
    }
  }

  // Обработка результата
  if (result) {
      // Тут код включения сигнала тревоги:
    tone(piezoPin, 2000); // Запустили звучание
  } else {
    // Тут код выключения сигнала тревоги:
    noTone(piezoPin); // Остановили звучание
  }
  
  smartdelay(1000);
}

// 
static int magic_method(float gpsFlat, float gpsFlon, float pointLat, float pointLon, float distance)
    {
        // магические константы. можно покрутить +-
        float lat_magic_number = 0.000009;
        float lon_magic_number = 0.000014;
        float lat_zone = lat_magic_number * distance;
        float lon_zone = lon_magic_number * distance;
        if ((fabs(gpsFlat - pointLat) < lat_zone) && (fabs(gpsFlon - pointLon) < lon_zone)) {
          return 1;
        } else {
          return 0;
        }
    }

// тестовая функция
// gpsFlat, gpsFlon - текущая широта, долгота с датчика
// pointLat, pointLon - Широта, долгота проверяемой точки
// return расстояние между точками
static float calculate(float gpsFlat, float gpsFlon, float pointLat, float pointLon)
{
    // пересчет в радианы
    float rGpsFlat = radians(gpsFlat);
    float rGpsFlon = radians(gpsFlon);

    float rPointLat = radians(pointLat);
    float rPointLon = radians(pointLon);

    // Большая (экваториальная) полуось
    float R = 6378245.0;

    return R * acos(sin(rGpsFlat) * sin(rPointLat) + cos(rGpsFlat) * cos(rPointLat) * cos(rGpsFlon - rPointLon));
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_float_debug(float val)
{
    int len = 15;
    int prec = 10;
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}
