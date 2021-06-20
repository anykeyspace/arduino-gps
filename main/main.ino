#include <NMEAGPS.h>

#include <math.h>

#define gpsPort Serial3

#define gsmPort Serial2

#define TONEpin 2 // Пин, к которому подключен пьезодинамик

#define PHONE "+ZZxxxxxxxxxx"

const int toneFreq = 4000; 

NMEAGPS gps;
gps_fix fix;

struct anshlagPoint
{
  float lat;
  float lon;
};

void setup()
{
  pinMode(TONEpin, OUTPUT);
  Serial.begin( 9600 );

  Serial.println("\n==== Tone example ====\n");
  beep();  delay(1000);
  beep2(); delay(1000);
  beep3(); delay(1000);
  beep4();
  Serial.println("End of tone example...\n");
  
  gpsPort.begin( 9600 );
  gsmPort.begin( 9600 );
  delay(1000);

  Serial.println("\n==== Sending test sms ====\n");
  sendTestSms();
  Serial.println("\n==== Sending completed ====\n");
}

void sendTestSms() {
  gsmPort.println("AT");
  updateSerial();

  gsmPort.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  gsmPort.print("AT+CMGS=\"");
  gsmPort.print(PHONE);
  gsmPort.print("\"");
  updateSerial();
  gsmPort.print("Arduino test sms"); //text content
  updateSerial();
  gsmPort.write(26);
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    gsmPort.write(Serial.read());//Forward what Serial received to gsmPort
  }
  while(gsmPort.available()) 
  {
    Serial.write(gsmPort.read());//Forward what gsmPort received to Serial Port
  }
}

void loop()
{
  if (gps.available( gpsPort )) {
    fix = gps.read();

    Serial.println( fix.latitude(), 6 ); 
    Serial.println( fix.longitude(), 6 );

    action();
  }
}

void action() {
  float flat = fix.location.latF();
  float flon = fix.location.lonF();
  
  // Проверяемые точки. Широта и долгота в градусах. Порядок не имеет значения
  int pointsNumber = 4; // количество точек
  struct anshlagPoint points[pointsNumber];
  points[0] = {53.190194, 45.064625};
  points[1] = {53.190194, 45.064625};
  points[2] = {53.190194, 45.064625};
  points[3] = {53.190194, 45.064625};

  // Радиус зоны обнаружения в метрах
  float distance = 10;

  //Проверка
  int result = 0;
  for (int i = 0; i < pointsNumber; ++i) {
    if (checkPoint(flat, flon, points[i].lat, points[i].lon, distance)) {
      result = 1;
      break;
    }
  }

  // Обработка результата
  if (result) {
    beep4(); // Звуковой сигнал, если попали в зону срабатывания
  }
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

static int checkPoint(float gpsFlat, float gpsFlon, float pointLat, float pointLon, float alertDistance)
{
  float distance = calculate(gpsFlat, gpsFlon, pointLat, pointLon);
  if (fabs(distance) < alertDistance) {
    return 1;
  }
  return 0;
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

// https://forum.arduino.cc/t/arduino-due-and-tone/133302/13

void beep()  {tone(50);}
void beep2() {beep(); delay(150); beep();}
void beep3() {beep(); delay(150); beep(); delay(150); beep();}
void beep4() {beep(); delay(150); beep(); delay(150); beep(); delay(150); beep();}

volatile static int32_t toggles;                    // number of ups/downs in a tone burst 

void tone(int32_t duration){                        // duration in ms
  const uint32_t rcVal = VARIANT_MCK/256/toneFreq;  // target value for counter, before it resets (= 82 for 4kHz)
  toggles = 2*toneFreq*duration/1000;               // calculate no of waveform edges (rises/falls) for the tone burst
  setupTC(TC1,0,TC3_IRQn,toneFreq);                 // Start Timer/Counter 1, channel 0, interrupt, frequency
  }
  
void setupTC(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq){
  pmc_set_writeprotect(false);                       // disable write protection of timer registers
  pmc_enable_periph_clk((uint32_t) irq);             // enable clock / interrupt 
//pmc_enable_periph_clk((uint32_t) ID_TC3);          // alternate syntax, using PMC_id instead
  TC_Configure(tc, channel,            
               TC_CMR_TCCLKS_TIMER_CLOCK4 |          // TIMER_CLOCK4: MCK/128=656,250Hz. 16 bits so 656,250/65,536=~10Hz/bit
               TC_CMR_WAVE |                         // Waveform mode
               TC_CMR_WAVSEL_UP_RC );                // Counter running up and reset when = Register C value (rcVal)
  const uint32_t rcVal = VARIANT_MCK/256/freq;       // target value for counter, before it resets
//Serial << "rcVal: " << rcVal << " toggles: " << toggles << '\n'; 
//TC_SetRA(tc, channel, rcVal/2);                    // could also use Register A for 50% duty cycle square wave
  TC_SetRC(tc, channel, rcVal);
  TC_Start(tc, channel);
  (*tc).TC_CHANNEL[channel].TC_IER =  TC_IER_CPCS;   // IER: CPCS bit enables RC compare interrupt when set 
  (*tc).TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;   // IDR: clear CPCS bit = don't disable RC compare interrupt
  NVIC_EnableIRQ(irq);                               // Enable TC3_IRQn in the Nested Vector Interrupt Controller)
  }

void TC3_Handler(void){                              // timer ISR  TC1 ch 0
  TC_GetStatus(TC1,0);
  if (toggles != 0){
    digitalWrite(TONEpin,!digitalRead(TONEpin));     // invert the pin state (i.e toggle)
    if (toggles > 0) toggles--;
    } 
  //else noTone();                                   // seems superfluous ?
  }
/*
void noTone(){
  TC_Stop(tc,channel);                               // stop timer
  digitalWrite(TONEpin,LOW);                         // no signal on pin
  }*/
