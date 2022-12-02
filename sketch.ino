// версия для резистора 430
// изменения от АК:
// 1. Cuise ON: длинное нажатие - ON, короткое нажатие - RES
// 2. сопротивление для ОК увеличено, дельта тоже
// 3. Вывод RING на Pioneer вместо моргания аварийкой
// 4. Добавлены кнопки для Pioneer через Ring (Bluetooth, Phone, Folder)
// 5. Обогрев вкл по PressCruiseRES, выкл по PressCruiseCAN
// 6. Маршкомп управленияется PressUp, PressDn, PressOK
// 7. Добавлена возможность долгого нажатия на все кнопки
// 8. Добавлена обратока долгого удержания кнопок с долгим нажатием

// версия
// 1.0 релиз
// 1.1 ребур через wdt
//     wdt в рабочем цикле
//     исправлена ложное срабатывание ATT при удержании
//     добавлен вывод версии прошивки
// 1.2 индикация ребута кнопками маршкомпа
//     понижена дельта из-за наложения кнопок set+ и left
//     откорректировны все входные сопротивления по результатам замеров
//     долгое нажатие изменено на 1.5 секунды
//     интеллектуальный контроль вкл-выкл максимальной мощности обогрева
// 1.3 индикация ребута переделана на мк ок дважды
//     частота ШИМ понижена до 30 гц, чтобы не жужжало
// 1.4 добавлены кнопки band/escape и BT connection
//     изменено выходное сопротивление Source
// 1.5 звуковое сопровождение включения и выключения обогрева 2мя пиками мк ок
//     ребут переделан на 4 пика мк ок
// 1.6 ШИМ реализован программно с частотой 2 Гц для нового транзистора
// 1.7 Включение круиза при нажатии любой кнопки первый раз после запуска
// 1.8 Увеличено время для вкл круиза с любой кнопки, изменено сопротивление кнопки PressSeekRight для Pioneer
// 1.9 Наконец-то сделана проверка включения круиз-контроля
// 1.10 Откорректировано сопротивление пассивного состояния для Pioneer
// 1.11 Отключено запоминание состояния обогрева руля
// 1.12 Добавлено отдельное включение круиза долгим удержанием кнопки ON
// 1.13 Ненажатая кнопка теперь через ring
// 1.14 Убраны двойные кнопки на магнитолу, почищен код от лишнего
// 1.15 Возвернуты кнопки управления звонками
//      Возвернута кнопка Voice Control
//      Добавлена функция выключения ESP
// 1.16 убрана функция выключения ESP
//      Добавлено вкл обогрева при старте по температуре
#define VERSION 1.16

/*
  =============================================================================================================
  КНОПКИ PIONEER                                            сопротивление     кнопки руля
  =============================================================================================================
  Source (переключение источников звука)                    1,2 КОм           PressOff
  АТТ (приглушение звука)                                   4,3 КОм           PressMode
  Display (управление режимом вывода информации на экран)   5,6 КОм           
  Трек вперёд                                               8,2 КОм           PressSeekRight
  Трек назад                                                11 КОм            PressSeekLeft
  Громкость вверх                                           16,4 КОм          PressVolUp
  Громкость вниз                                            22 КОм            PressVolDn
  band/escape                                               50 КОм            длинное нажатие PressMode

  Вход в меню Bluetooth (в список контактов для звонка)     1,2 Ом + Ring     
  Phone принять звонок                                      4,3 КОм + Ring    PressSeekLeft
  Phone завершить звонок (hold - отклонить звонок)          5,6 КОм + Ring    PressSeekRight
  Перелистывание папок вперёд                               8.2 КОм + Ring    PressRight
  Перелистывание папок назад                                12 КОм + Ring     PressLeft
  =============================================================================================================
  КНОПКИ МК                                                                   кнопки руля 
  =============================================================================================================
  Вверх                                                                       PressUp
  Вниз                                                                        PressDn
  Выбор                                                                       PressOk
  Вкл/выкл ESP                                                                длинное нажатие PressOk
  =============================================================================================================
  КНОПКИ КРУЗИА                                                               кнопки руля
  =============================================================================================================
  Включение                                                                   длинное нажатие PressCruiseOn
  RES                                                                         PressCruiseOn
  SET+                                                                        PressCruiseUp
  SET-                                                                        PressCruiseDn
  =============================================================================================================
  ОБОГРЕВ РУЛЯ
  =============================================================================================================
  вкл                                                                         PressCruiseRES
  выкл                                                                        PressCruiseCAN
  =============================================================================================================
  Программный ресет                                                           длинное нажатие PressCruiseCAN
  =============================================================================================================
*/


#include <EEPROM.h>
#include <SPI.h>

// чтение кнопок руля FF3 restyle
//        no_press, vol+, vol-, seek-, seek+, voice, on, res, can, delta
int audio[] = {775, 383,  247,   618,   510,   113,  688, 548, 3,   10};

//        no_press, up, down, left, right, ok, cr off, set+, set-, delta
int marsh[] = {836, 292, 429, 552,  643,   138, 6,     733,  530,  10};

// коды сопротивлений микросхемы MCP
#define MCP_Source        12 //10
#define MCP_att           24//22
#define MCP_display       44
#define MCP_answer        24
#define MCP_hangup        44
#define MCP_seekLeft      65//57
#define MCP_seekRight     52//41
#define MCP_VolUp         92//84
#define MCP_VolDown       125
#define MCP_no            178//172
#define MCP_band          255

#define delayHeat         90000 // задержка включения обогрева в половину мощности
#define HEAT_OFF_POWER    0
#define HEAT_HALF_POWER   700
#define HEAT_MAX_POWER    1023

#define PROG_PWM_PULSE    1400
#define PROG_PWM_PERIOD   2000

#define marshPin     A7
#define audioPin     A6

int i = 0;
int prevButton = 0;
int csPin = 10;      // CS на цифровом потенциометре

#define PressOk         1
#define PressUp         2
#define PressDn         3
#define PressLeft       4
#define PressRight      5
#define PressNo         6

#define PressMode       7
#define PressVolUp      8
#define PressVolDn      9
#define PressSeekLeft   10
#define PressSeekRight  11

#define PressCruiseOn   16
#define PressCruiseUp   17
#define PressCruiseDn   18
#define PressCruiseRes  20
#define PressCruiseCan  21
#define PressCruiseOff  22

#define out_cruise_on       2 // круиз он
#define out_cruise_res      3 // круиз рес
#define out_cruise_up       4 // круиз +
#define out_cruise_dn       5 // круиз -
#define out_mk_up           6 // маршкомп +
#define out_mk_dn           7 // маршкомп -
#define out_mk_ok           8 // маршкомп ок
#define out_ring            12 // ring аудио
#define out_heating         9 // обогрев руля

#define shortPress  700     // макс длительность короткого нажатия миллисекунд
#define holdPress   1500    // мин длительность длинного нажатия миллисекунд

#define HEAT_ON     11
#define HEAT_OFF    10

#define TIMER_START_HEAT_BY_TEMP 20000 // 20 секунд

byte HeatState = HEAT_OFF;      // значение состояния обогрева
boolean HeatTimer = false;      // флаг таймера включения обогрева на половину мощности
boolean HeatStartTimer = false; // флаг таймера включения обогрева по температуре
unsigned long timeStamp = 0;      // метка времени включения обогрева
unsigned long bTime = 0;          // метка времени длительности нажания кнопки
int currButton;

unsigned long pwm_timer = 0;

bool CruiseIsOn = false;

void (* reboot)(void) = 0;  // вызов для программного ресета
void heating_change_state(boolean on);

byte test_res = 28;
//=========================================================================================================================
void MCP41010Write(byte value)
{
  digitalWrite(csPin, LOW);
  SPI.transfer(B00010001);    // cmd to set
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
}

//=========================================================================================================================
void outs_reset()
{
  digitalWrite(out_cruise_on, LOW);
  digitalWrite(out_cruise_res, LOW);
  digitalWrite(out_cruise_up, LOW);
  digitalWrite(out_cruise_dn, LOW);
  digitalWrite(out_mk_dn, LOW);
  digitalWrite(out_mk_up, LOW);
  digitalWrite(out_mk_ok, LOW);
  digitalWrite(out_ring, HIGH);
}

//=========================================================================================================================
void setup()
{
  double temp;
  unsigned int wADC;

  Serial.begin(9600);

  Serial.println("FF3 steering wheel buttons controller with Pioneer");
  Serial.print("Version ");
  Serial.println(VERSION);

  HeatState = HEAT_OFF;   // значение состояния обогрева
  HeatTimer = false;      // флаг таймера включения обогрева

  outs_reset();
  digitalWrite(out_heating, LOW);

  pinMode(out_cruise_on, OUTPUT);
  pinMode(out_cruise_res, OUTPUT);
  pinMode(out_cruise_up, OUTPUT);
  pinMode(out_cruise_dn, OUTPUT);
  pinMode(out_mk_ok, OUTPUT);
  pinMode(out_mk_up, OUTPUT);
  pinMode(out_mk_dn, OUTPUT);
  pinMode(out_ring, OUTPUT);
  pinMode(out_heating, OUTPUT);

  outs_reset();
  digitalWrite(out_heating, LOW);

  pinMode(csPin, OUTPUT);
  delay(50);
  SPI.begin();
  delay(50);
  SPI.transfer(0);  // команда
  SPI.transfer(0);  // значение
  MCP41010Write(MCP_no);

  pinMode(marshPin, INPUT);
  pinMode(audioPin, INPUT);

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

// Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  // offset = wADC - temp * 1.2
#define ADC_TEMP_OFFSET 324.31
  temp = (wADC - ADC_TEMP_OFFSET) / 1.22;
  Serial.println(temp);
  Serial.println(wADC);

  if(temp < 4.0)
  {// холодно, заводим таймер на включение обогрев руля
    HeatStartTimer = true;
  }
}


//=========================================================================================================================
int getR()  // чтение сопротивления кнопок на руле и возврат нажатой кнопки или 0
{
  int r = analogRead(marshPin);
  int a = analogRead(audioPin);

  // Marsh
  if ((r > (marsh[1] - marsh[9])) && (r < (marsh[1] + marsh[9]))) return (PressUp);
  if ((r > (marsh[2] - marsh[9])) && (r < (marsh[2] + marsh[9]))) return (PressDn);
  if ((r > (marsh[3] - marsh[9])) && (r < (marsh[3] + marsh[9]))) return (PressLeft);
  if ((r > (marsh[4] - marsh[9])) && (r < (marsh[4] + marsh[9]))) return (PressRight);
  if ((r > (marsh[5] - 30)) && (r < (marsh[5] + 30)))             return (PressOk);

  // Cruise Left
  if ((r > (marsh[6] - marsh[9])) && (r < (marsh[6] + marsh[9]))) return (PressCruiseOff);
  if ((r > (marsh[7] - marsh[9])) && (r < (marsh[7] + marsh[9]))) return (PressCruiseUp);
  if ((r > (marsh[8] - marsh[9])) && (r < (marsh[8] + marsh[9]))) return (PressCruiseDn);

  // Audio
  if ((a > (audio[1] - audio[9])) && (a < (audio[1] + audio[9]))) return (PressVolUp);
  if ((a > (audio[2] - audio[9])) && (a < (audio[2] + audio[9]))) return (PressVolDn);
  if ((a > (audio[3] - audio[9])) && (a < (audio[3] + audio[9]))) return (PressSeekLeft);
  if ((a > (audio[4] - audio[9])) && (a < (audio[4] + audio[9]))) return (PressSeekRight);
  if ((a > (audio[5] - audio[9])) && (a < (audio[5] + audio[9]))) return (PressMode);

  // Cruise Right
  if ((a > (audio[6] - audio[9])) && (a < (audio[6] + audio[9]))) return (PressCruiseOn);
  if ((a > (audio[7] - audio[9])) && (a < (audio[7] + audio[9]))) return (PressCruiseRes);
  if ((a > (audio[8] - audio[9])) && (a < (audio[8] + audio[9]))) return (PressCruiseCan);

  return 0; // если ничего не нажато
}

//=========================================================================================================================
void heating_change_state(boolean on)
{
  if (on == false)
  { // выкл обогрев
    if(HeatState == HEAT_ON)
    {
      if(HeatTimer == false)
      {// обогрев уже работает в половину, значит давно греем
        HeatTimer = true;
        timeStamp = millis();            // начало отсчета времени до разрешения максимальной мощности при след. включении
      }
      else
      {// обогрев еще не закончил фазу разогрева на макс мощности, в след. вкл. сразу на полную будет опять
        HeatTimer = false;
      }
      digitalWrite(out_heating, LOW);   
      HeatState = HEAT_OFF;
      Serial.println("Выкл обогрев руля");
    }
    // пикнем что выключили
    digitalWrite(out_mk_ok, HIGH);
    delay(100);
    digitalWrite(out_mk_ok, LOW);
    delay(200);
    digitalWrite(out_mk_ok, HIGH);
    delay(100);
    digitalWrite(out_mk_ok, LOW);
  }
  else
  { // вкл обогрев
    if(HeatState != HEAT_ON)
    {
      if((HeatTimer == true) && ((millis() - timeStamp) < delayHeat))
      {// прошло мало времени с момента выключения обогрева
        HeatTimer = false;
        digitalWrite(out_heating, LOW);
        Serial.println("Вкл обогрев руля на ПОЛОВИНУ");
      }
      else
      {
        HeatTimer = true;
        timeStamp = millis();                       // начало отсчета времени до снижения тока обогрева
        digitalWrite(out_heating, HIGH);
        Serial.println("Вкл обогрев руля на МАКСИМУМ");
      }
      HeatState = HEAT_ON;
    }
    // пикнем что включили
    digitalWrite(out_mk_ok, HIGH);
    delay(100);
    digitalWrite(out_mk_ok, LOW);
    delay(200);
    digitalWrite(out_mk_ok, HIGH);
    delay(100);
    digitalWrite(out_mk_ok, LOW);
  }
}

//=========================================================================================================================
void Check_Cruise_On(void)
{
  if(false == CruiseIsOn)
  {
    CruiseIsOn = true;
    // включаем круиз
    digitalWrite(out_cruise_on, HIGH);
    delay(100);
    Serial.println("Cruise on");
    digitalWrite(out_cruise_on, LOW);
    delay(400);
  }
}

//=========================================================================================================================
void singleButton(void)
{
  switch (currButton)
  {
    // --------- PIONEER ---------
    case PressVolUp:
      digitalWrite(out_ring, LOW);
      MCP41010Write(MCP_VolUp);
      Serial.println("Volume+");
    break;

    case PressVolDn:
      digitalWrite(out_ring, LOW);
      MCP41010Write(MCP_VolDown);
      Serial.println("Volume-");
    break;

    case PressCruiseOff:
      digitalWrite(out_ring, LOW);
      MCP41010Write(MCP_Source);
      Serial.println("Source");
    break;

    case PressRight:
      digitalWrite(out_ring, HIGH);
      MCP41010Write(MCP_seekRight);
      Serial.println("Folder+");
    break;

    case PressLeft:
      digitalWrite(out_ring, HIGH);
      MCP41010Write(MCP_seekLeft);
      Serial.println("Folder-");
    break;
          
    case PressSeekLeft:
      digitalWrite(out_ring, LOW);
      MCP41010Write(MCP_seekLeft);
      Serial.println("SeekLeft");
      delay(200);
      digitalWrite(out_ring, HIGH);
      MCP41010Write(MCP_answer);
      Serial.println("Answer");
      delay(200);
    break;

    case PressSeekRight:
      digitalWrite(out_ring, LOW);
      MCP41010Write(MCP_seekRight);
      Serial.println("SeekRight");
      delay(200);
      digitalWrite(out_ring, HIGH);
      MCP41010Write(MCP_hangup);
      Serial.println("Hangup");
      delay(200);
    break;
 
    // --------- MK ---------
    case PressUp:
      digitalWrite(out_mk_up, HIGH);
      Serial.println("MK Up");
    break;

    case PressDn:
      digitalWrite(out_mk_dn, HIGH);
      Serial.println("MK Dn");
    break;
    
    // --------- CRUISE --------- 
    case PressCruiseUp:
      Check_Cruise_On();
      digitalWrite(out_cruise_up, HIGH);
      delay(200);
      Serial.println("Cruise Set+");
    break;
  
    case PressCruiseDn:
      Check_Cruise_On();
      digitalWrite(out_cruise_dn, HIGH);
      delay(200);
      Serial.println("Cruise Set-");
    break;
    
    // --------- HEATING ---------
    case PressCruiseRes:
      heating_change_state(true);
      Serial.println("Heating On");
    break;
    
    // --------- НЕТ НАЖАТИЯ ---------
    case 0:
        MCP41010Write(MCP_no);
        outs_reset();
    break;

    default: break;
  }
}

//=========================================================================================================================
void doubleButton(void)
{
  if (bTime < shortPress)
  { // ===================================== короткое нажатие =============================================
    switch (prevButton)
    {
      // --------- MK ---------
      case PressOk:
        digitalWrite(out_mk_ok, HIGH);
        Serial.println("MK Ok");
      break;
      
      // --------- CRUISE ---------
      case PressCruiseOn:
        Check_Cruise_On();
        digitalWrite(out_cruise_res, HIGH);
        delay(100);
        Serial.println("Cruise RES");
      break;

      // --------- PIONEER ---------
      case PressMode:
        digitalWrite(out_ring, LOW);
        MCP41010Write(MCP_att);
        Serial.println("ATT");
      break;
    
      // --------- HEATING ---------
      case PressCruiseCan:
        heating_change_state(false);
        Serial.println("Heating Off");
      break;

      default: break;
    }
  }
  else
  { // ========================================= долгое нажатие ============================================
    switch (prevButton)
    {
      // --------- MK ---------
      /*case PressOk:
        // вниз до меню
        digitalWrite(out_mk_up, HIGH);
        delay(100);
        digitalWrite(out_mk_up, LOW);
        delay(100);
        digitalWrite(out_mk_up, HIGH);
        delay(100);
        digitalWrite(out_mk_up, LOW);
        delay(100);
        digitalWrite(out_mk_up, HIGH);
        delay(100);
        digitalWrite(out_mk_up, LOW);
        delay(100);
        // вход в меню
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(100);
        // вкл/выкл ESP
	       digitalWrite(out_mk_dn, HIGH);
        delay(100);
        digitalWrite(out_mk_dn, LOW);
        delay(100);
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(100);
        // выход из меню
        digitalWrite(out_mk_up, HIGH);
        delay(100);
        digitalWrite(out_mk_up, LOW);
        delay(100);
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(100);
        // назад к мгновенному расходу
        digitalWrite(out_mk_dn, HIGH);
        delay(100);
        digitalWrite(out_mk_dn, LOW);
        delay(100);
        digitalWrite(out_mk_dn, HIGH);
        delay(100);
        digitalWrite(out_mk_dn, LOW);
        delay(100);
        digitalWrite(out_mk_dn, HIGH);
        delay(100);
        digitalWrite(out_mk_dn, LOW);
        delay(100);
        Serial.println("ESP on\off");
      break;*/
      
      // --------- CRUISE ---------
      case PressCruiseOn:
        digitalWrite(out_cruise_on, HIGH);
        Serial.println("Cruise ON");
      break;

      // --------- PIONEER ---------
      case PressMode:
        digitalWrite(out_ring, HIGH);
        MCP41010Write(MCP_Source);
        delay(100);
        Serial.println("BT menu");
      break;
    
      // --------- REBOOT ---------
      case PressCruiseCan:
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(200);
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(200);
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        delay(200);
        digitalWrite(out_mk_ok, HIGH);
        delay(100);
        digitalWrite(out_mk_ok, LOW);
        reboot();
      break;

      default: break;
    }
  }
}

//=========================================================================================================================
void program_PWM()
{
  if((millis() - pwm_timer) > PROG_PWM_PERIOD)
  {// период прошел
    pwm_timer = millis();
    digitalWrite(out_heating, HIGH);
  }
  else if((millis() - pwm_timer) > PROG_PWM_PULSE)
  {// длительность импульса прошла
    digitalWrite(out_heating, LOW);
  }
}

//=========================================================================================================================
void loop() {
  //int r = analogRead(marshPin);int a = analogRead(audioPin);
  //Serial.print("MK="); Serial.print(r); Serial.print("Aud="); Serial.println(a);
  
  currButton = getR(); // читаем код кнопки
  delay(10);
  byte bounceButton = getR(); // еще раз читаем
  delay(10);
  byte bounceButton2 = getR(); // еще раз читаем

  if ((currButton == bounceButton) && (currButton == bounceButton2))
  { // дребезга нет

    // ======================= одиночные кнопки ======================
    singleButton();

    // ======================= двойные кнопки ========================
    if ((currButton != 0) && (prevButton == 0))
    { // кнопка нажата 
      bTime = millis();
    }
    
    if ((prevButton != 0) && (currButton == 0))
    { // кнопка отпущена
      bTime = millis() - bTime; // вычисление время удержания кнопки
      doubleButton();
      delay(100); // чтобы сразу не выкл в следующем цикле
    }
    
    prevButton = currButton;
  }
  
  if((HeatTimer == true) && (HeatState == HEAT_ON))
  { // ============ таймер обогрева руля =============
    if((millis() - timeStamp) > delayHeat)           // вычисление задежки по таймеру
    {
      HeatTimer = false;
      //analogWrite(out_heating, HEAT_HALF_POWER);      // по окончанию времени таймера обогрев на половину
      digitalWrite(out_heating, LOW);
      Serial.println("Вкл обогрев руля на ПОЛОВИНУ мощности");
    }
  }

  if((HeatState == HEAT_ON) && (HeatTimer == false))
  {
    program_PWM();
  }

  if(HeatStartTimer && (TIMER_START_HEAT_BY_TEMP == millis()))
  {// холодно, включаем обогрев руля после задержки
    HeatStartTimer = false;
    heating_change_state(true);
  }
}
