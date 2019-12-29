//08 12 2019 - 27
#define _DEBAG_ 0 // !!!
#define _UGOL_ 0
#define _PRINT_BAT_ 0
#define _LED13_ 0



#define _FOTO_DIF_1 100
#define _FOTO_DIF_2 100
#define _FOTO_DIF_3 100
#define _FOTO_DIF_4 100
#define _FOTO_DIF_5 100
#define _FOTO_DIF_6 100

#define _DEBOUNCE_R 10// настройка антидребезга (по умолчанию 80 мс)
#define _TIME_OUT_R 1000// настройка таймаута на удержание (по умолчанию 500 мс)
#define _CLICK_TIME_R 600// настройка таймаута между кликами (по умолчанию 300 мс)
#define _SPEED_SERVO_R 200//установка максимальной скорости (условные единицы, 0 – 200)
#define _ACCEL_SERVO_R 1//становка ускорения (0.05 – 1). При значении 1 ускорение максимальное
#define _DO_NALIV 100 //задержка до налива мс
#define _POSLE_NALIV 200 //задержка после налива мс
#define _LED_OFF 15000 //задержка на отключение светодиодов !!!
#define _POW_OFF 15000 //задержка на отключение светодиодов !!!
#define _LED_OFF_A 15000 //задержка на отключение светодиодов !!!
#define _POW_OFF_A 15000 //задержка на отключение светодиодов !!!


#define _EPP_CON 88 //


#define _ARDUINO_PIT 5.01 //5.01 ножка питания ардуино
#define BTN_ENT_PIN 2// пин кнопки ввод
#define BTN_UP_PIN 3// пин кнопки +
#define BTN_DN_PIN 4// пин кнопки -
#define PIN_SERVO 7
#define PIN_SERVO_ON 6
#define PIN_PUMP_ON 9

//#define _B 0
#define _R 1
#define _G 2
#define _B 3
#define _W 4



#include "GyverButton.h"
GButton buttEnt(BTN_ENT_PIN);
GButton buttUp(BTN_UP_PIN);
GButton buttDn(BTN_DN_PIN);

#include "ledEf.h"
#include "simbols1.h"
#include "MatrixCascade.h"
MatrixCascade<2> cascade(12, 11, 10);

#include "GyverTimer.h"
GTimer myTimer(MS);    // создать миллисекундный таймер
GTimer myTimer2(MS);    // создать миллисекундный таймер

#include "ServoSmooth.h"
ServoSmooth servo;

#include <EEPROM.h>

int igUst[6] {972, 903, 755, 716, 574, 876};
int igUstDif[6] {_FOTO_DIF_1, _FOTO_DIF_2, _FOTO_DIF_3, _FOTO_DIF_4, _FOTO_DIF_5, _FOTO_DIF_6};
byte bygPosDeg[6] {0, 37, 63, 97, 129, 180};
byte bygRum[6];
byte bygRumRuch[6] {1, 0, 0, 0, 0, 0};
byte bygRumLast[6] {1, 1, 1, 1, 1, 1};
unsigned long ulTimePamp[3] = {100, 200, 300};
byte bygDoza = 2;
byte bygAutoDoza =  0;
byte bygPosN;
byte bygLastStatus = 20;
byte bygStatus = 0;
byte bygTemp;
byte bygTemp1;
byte bygPosNaliv = 0;

void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, 0);
  pinMode(PIN_SERVO_ON, OUTPUT);
  pinMode(PIN_PUMP_ON, OUTPUT);
  digitalWrite(PIN_PUMP_ON, 0);
  digitalWrite(PIN_SERVO_ON, 1);

#if(_LED13_)
#pragma message "_LED13_"
  pinMode(13, OUTPUT);
#endif

#if(_LED13_)
#endif

#if(_DEBAG_)
#pragma message "D_E_B_A_G"
  Serial.begin(9600);
#endif
#if(_DEBAG_)
  Serial.println("_DEBAG_");
#endif


  EEPROM.get(12, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetFotoSens();
  } else {
#if(_DEBAG_)
    Serial.println("eep Foto !");
#endif
  }
  EEPROM.get(51, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetFotoSensR();
  } else {
#if(_DEBAG_)
    Serial.println("eep Diff !");
#endif
  }
  EEPROM.get(25, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetServoPos();
  } else {
#if(_DEBAG_)
    Serial.println("eep servo !");
#endif
  }
  EEPROM.get(38, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetTimePomp();
  } else {
#if(_DEBAG_)
    Serial.println("eep Pump !");
#endif
  }

  EEPROM.get(54, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetbygRum();
    EEPROM.get(52, bygLastStatus);
    EEPROM.get(60, bygDoza);
#if (_DEBAG_)
    Serial.println("Статус");
    Serial.println(bygLastStatus);
    Serial.println("Доза");
    Serial.println(bygDoza);
#endif
  } else {
#if(_DEBAG_)
    Serial.println("eep Stat !");
#endif
  }
  EEPROM.get(73, bygTemp);
  if (bygTemp == _EPP_CON) {
    vGetbygRumRuch();
  } else {
#if(_DEBAG_)
    Serial.println("eep Руч !");
#endif
  }

  cascade[0].setIntensity(0);//яркость матрицы
  cascade[1].setIntensity(15);//яркость светодиодов стола
  cascade[0].setRotation(2);
  cascade.clear();
  //  cascade[1].setRotation(1);

  buttEnt.setDebounce(_DEBOUNCE_R);// настройка антидребезга (по умолчанию 80 мс)
  buttUp.setDebounce(_DEBOUNCE_R);// настройка антидребезга (по умолчанию 80 мс)
  buttDn.setDebounce(_DEBOUNCE_R);// настройка антидребезга (по умолчанию 80 мс)

  buttEnt.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)
  buttUp.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)
  buttDn.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)

  // HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC (BTN_PIN --- КНОПКА --- GND)
  // LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  // по умолчанию стоит HIGH_PULL
  buttEnt.setType(HIGH_PULL);
  buttUp.setType(HIGH_PULL);
  buttDn.setType(HIGH_PULL);

  // NORM_OPEN - нормально-разомкнутая кнопка
  // NORM_CLOSE - нормально-замкнутая кнопка
  // по умолчанию стоит NORM_OPEN
  buttEnt.setDirection(NORM_OPEN);
  buttUp.setDirection(NORM_OPEN);
  buttDn.setDirection(NORM_OPEN);

  servo.attach(PIN_SERVO, 400, 2600); // 600 и 2400 - длины импульсов, при которых
  // серво поворачивается максимально в одну и другую сторону, зависят от самой серво
  // и обычно даже указываются продавцом. Мы их тут указываем для того, чтобы
  // метод setTargetDeg() корректно отрабатывал диапазон поворота сервы

  servo.setSpeed(_SPEED_SERVO_R);   // установка максимальной скорости (условные единицы, 0 – 200)
  servo.setAccel(_ACCEL_SERVO_R);  // становка ускорения (0.05 – 1). При значении 1 ускорение максимальное

  servo.setAutoDetach(false); // отключить автоотключение (detach) при достижении целевого угла (по умолчанию включено)
}
void loop() {
  buttEnt.tick();  // обязательная функция отработки. Должна постоянно опрашиваться
  buttUp.tick();  // обязательная функция отработки. Должна постоянно опрашиваться
  buttDn.tick();  // обязательная функция отработки. Должна постоянно опрашиваться

  switch (bygStatus) {
    case 0:
      if (buttEnt.state()) {
        bygStatus = 9;
      } else bygStatus = 80;

      break;
    case 9:
      digitalWrite(5, 0);
      cascade.clear();
      vLedEf2(LedData2, 1);
      bygStatus = 10;
      break;
      break;
    case 10:
      if (vLedEf2(LedData2, 0)) {
        if (bygLastStatus == 0) {
          bygLastStatus = 20;
        }
        bygStatus = bygLastStatus;
      }
      vPrintCapBat();
      break;
    case 20://*************************************************************** 20 РЮМКА (Доза)
#if(_DEBAG_)
      Serial.println("20-Доза");
#endif
      vPrintDoza(18, 32);
      vSensRumsRefresh();
      buttDn.isHolded();
      buttUp.isHolded();
      buttEnt.isClick();
      buttDn.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)
      bPovorot(bygPosDeg[0]);
      myTimer.setTimeout(_LED_OFF);
      bygStatus = 21;
      break;
    case 21://*************************************************************** 20 РЮМКА (Доза)
#if(_LED13_)
      //      digitalWrite(13, myTimer.isEnabled());
      //      digitalWrite(13, buttDn.state());
#endif
      if (myTimer.isReady()) {
        bygLastStatus = 20;
        bygStatus = 140;// на сон
#if(_DEBAG_)
        Serial.println("на сон");
#endif
      }
      if ( bSensRums()) {
        myTimer.setTimeout(_LED_OFF);
#if(_DEBAG_)
        Serial.println("Движение");
#endif
      }
      if (buttUp.isClick() ) {
        // vSensRumsRefresh();
        myTimer.setTimeout(_LED_OFF);
        if (bygDoza < 4) {
          bygDoza++;
        }
        vPrintDoza(18, 32);
      }
      if (buttDn.isClick()  ) {
        // vSensRumsRefresh();
        myTimer.setTimeout(_LED_OFF);
        if (bygDoza > 0) {
          bygDoza--;
        }
        vPrintDoza(18, 32);
      }
      if (buttDn.isHolded()) {
        bygStatus = 70;//
      }
      if (buttUp.isHolded()) {
        bygStatus = 100;// на автоналив
      }
      if (buttEnt.isClick()) {
        bygStatus = 40;// на налив
      }
      if (buttEnt.isHolded()) {
        bygRum[0] = 0;
        bygRum[1] = 0;
        bygRum[2] = 0;
        bygRum[3] = 0;
        bygRum[4] = 0;
        bygRum[5] = 0;
      }
      break;
    case 30://********************************************************************** 30  калибровка фтотодатчиков
      fPrintChar(17);//Ф
      bygStatus = 31;
      buttEnt.isHolded();
      buttUp.isHolded();
      buttEnt.isClick();
      vSensRumsRefresh();
      buttDn.setTimeout(_TIME_OUT_R); // настройка таймаута на удержание (по умолчанию 500 мс)
      bPovorot(bygPosDeg[0]);
      break;
    case 31://----------------------------------------------------------------------31  калибровка фтотодатчиков
      bSensRums();
      if (buttEnt.isClick()) {
        vTuneFotosens();
        vSaveFotoSens();
        cascade[0].on(7, 0);
      }
      if (buttUp.isHolded()) {
        bygStatus = 70;
      }
      if (buttDn.isHolded()) {
        bygStatus = 60;
      }
      if (buttEnt.isHolded()) {
        vTuneFotosensR();
        vSaveFotoSensR();
        cascade[0].off(7, 0);
      }
      break;
    case 40:// ****************************************************** 40 налив
      vPrintCapBat();
      vNaliv(bygDoza);
      bygStatus = 20;// на рюмку
      break;

    case 50://******************************************************** 50 калибровка серво
      cascade.clear();
      fPrintChar(22);//c
      bygStatus = 51;
      buttUp.isHolded();
      buttDn.isHolded();
      buttEnt.isHolded();
      buttEnt.isClick();
      buttDn.isClick();
      buttUp.isClick();
      bygPosN = 0;
      cascade[1].on(1, (bygPosN + 1)); // зеленый
      bPovorot(bygPosDeg[0]);
      break;
    case 51:// -----------------------------------------------------------51 калиб. серво
      vCalibServo();
      if (buttUp.isHolded()) {
        bPovorot(bygPosDeg[0]);
        bygStatus = 60;// на калиб. насос
      }
      if (buttDn.isHolded()) {
        bPovorot(bygPosDeg[0]);
        bygStatus = 20;// на рюмку
      }
      break;
    case 60://************************************************************ 60 Калибровка насоса
      fPrintChar(23);// н
      cascade[0].on(7, 0);
      buttEnt.setDebounce(10);// настройка антидребезга
      bygStatus = 61;
      buttUp.isHolded();
      buttUp.isClick();
      buttDn.isClick();
      buttEnt.isClick();
      vSensRumsRefresh();
      bygPosN = 0;
      bPovorot(bygPosDeg[0]);
      break;
    case 61://Калибровка насоса
      bSensRums();
      vPumpCalib();
      if (buttUp.isHolded()) {
        buttEnt.setDebounce(_DEBOUNCE_R);// настройка антидребезга
        bygStatus = 30;// на фотодатчики
      }
      if (buttDn.isHolded()) {
        buttEnt.setDebounce(_DEBOUNCE_R);// настройка антидребезга
        bygStatus = 50;// на серво
      }
      break;
    case 70://************************************************************** 70 Промывка
      vSensRumsRefresh();
      vPrintCapBat();
      bygStatus = 71;
      buttDn.isHolded();
      buttDn.isClick();
      buttUp.isClick();
      buttUp.isHolded();
      myTimer.setTimeout(5000);   // настроить таймаут
      buttDn.setTimeout(3000);        // настройка таймаута на удержание (по умолчанию 500 мс)
      break;
    case 71://Промывка
      bSensRums();
      bygStatus = DelayWithSensRum(bygStatus, 20);
      if (buttUp.isClick() or buttUp.isHolded()) {
        bygStatus = 20;// на рюмку
      }
      if (buttDn.isHolded()) {
        buttEnt.isClick();
        myTimer.setTimeout(5000);   // настроить таймаут
        bygStatus = 30;// на калиб фото
      }
      if (buttEnt.state()) {
        if ( bygRum[0] != 0) {
          digitalWrite(PIN_PUMP_ON, 1);
          bygRum[0] = 2;
        }
        myTimer.setTimeout(5000);   // настроить таймаут
      } else digitalWrite(PIN_PUMP_ON, 0);
#if(_LED13_)
      digitalWrite(13, myTimer.isEnabled());
#endif
      break;
    case 80://Зарядка ******************************************************* 80 Зарядка
      cascade.clear();
      bygStatus = 81;
      vPrintCapBat();//   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      digitalWrite(5, 1);
      myTimer.setInterval(300);
      bygTemp = 0;
      bygTemp1 = 0;

      break;
    case 81://Зарядка ******************************************************* 81 Зарядка

      if (buttEnt.state() || buttUp.state() || buttDn.state()) {
        bygStatus = 9;
      }
      if (myTimer.isReady() ) {
        bygTemp ++;
        if (bygTemp == 1) {
          cascade[0].setRow(0, B10111101);
        }
        if (bygTemp == 2) {
          cascade[0].setRow(0, B00111100);
        }
        if (bygTemp > 2) {
          bygTemp = 0;
          bygTemp1++;
        }
        if (bygTemp1 == 9) {
          digitalWrite(5, 0);
        }
        if (bygTemp1 > 10) {
          bygTemp1 = 0;
          vPrintCapBat();//   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          digitalWrite(5, 1);
        }

      }

      break;
    case 90://****************************************************************90 Световые эфекты
      buttUp.isClick();
      buttDn.isClick();
      fPrintChar(LedEfNum(buttUp.isClick(), buttDn.isClick()));// 0 - 9
      bygStatus = 91;
      buttDn.isHolded();
      buttEnt.isClick();
      break;
    case 91://*****************************************************************91 Световые эфекты
      int iNumEf ;
      if (buttDn.isHolded()) {
        bygStatus = 130;
      }
      if (buttUp.isHolded()) {
        bygStatus = 20;
      }
      LedEfNum(buttUp.isClick(), buttDn.isClick());
      if  ( buttEnt.isClick()) {
        iNumEf = LedEfNum(buttUp.isClick(), buttDn.isClick());
      }
      vLedEfSel(iNumEf);
      break;
    case 100://**************************************************************** автоналив
      vPrintDoza(24, 34);
      vSensRumsRefresh();
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      bygTemp = 0;
      myTimer.stop();
      myTimer2.setTimeout(_LED_OFF_A);
      bygStatus = 101;
      break;
    case 101://***************************************************************************** автоналив
      if (myTimer2.isReady()) {
        bygLastStatus = 100;
        bygStatus = 140;// на сон
#if(_DEBAG_)
        Serial.println("на сон");
#endif
      }
      if (bSensRums()) {
        if ( (bygRum[0] == 1) || (bygRum[1] == 1) || (bygRum[2] == 1) || (bygRum[3] == 1) || (bygRum[4] == 1) || (bygRum[5] == 1)) {
          myTimer.setTimeout(300);   // настроить таймаут
        } else {
          myTimer.stop();
          vPrintDoza(24, 34);
        }
        bygTemp = 0;
        myTimer2.setTimeout(_LED_OFF_A);
      }
      if (myTimer.isReady()) {
        myTimer2.setTimeout(_LED_OFF_A);
        fPrintChar(bygTemp);
        bygTemp++;
        if ( (bygRum[0] == 1) || (bygRum[1] == 1) || (bygRum[2] == 1) || (bygRum[3] == 1) || (bygRum[4] == 1) || (bygRum[5] == 1)) {
          myTimer.setTimeout(300);   // настроить таймаут
        } else {
          myTimer.stop();
          bygTemp = 0;
          vPrintDoza(24, 34);
        }
        if (bygTemp > 10) {
          bygStatus = 102;
          myTimer.stop();

        }
      }

      if (buttEnt.isClick() ) {
        vPrintCapBat();
        vNaliv(bygDoza);
        vPrintDoza(24, 34);
        myTimer2.setTimeout(_LED_OFF_A);
      }
      if (buttUp.isClick() ) {
        myTimer2.setTimeout(_LED_OFF_A);
        if (bygDoza < 4) {
          bygDoza++;
          vPrintDoza(24, 34);
        }
      }
      if (buttDn.isClick()  ) {
        myTimer2.setTimeout(_LED_OFF_A);
        if (bygDoza > 0) {
          bygDoza--;
          vPrintDoza(24, 34);
        }
      }

      if (buttDn.isHolded()) {
        bygStatus = 20;
      }
      if (buttUp.isHolded()) {
        bygStatus = 120;
      }
      break;
    case 102:
      vPrintCapBat();
      vNaliv(bygDoza);
      bygStatus = 100;
      break;
    case 120://**************************************************************** ручной налив
      vRuchLeds();
      bPovorot(bygPosDeg[0]);
      vPrintDoza(32, 36);
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      vRuchLeds();
      myTimer.setTimeout(_LED_OFF);
      bygStatus = 121;
      break;
    case 121://**************************************************************** ручной налив
      if (myTimer.isReady()) {
        bygLastStatus = 120;
        bygStatus = 140;// на сон
#if(_DEBAG_)
        Serial.println("на сон");
#endif
      }
      if (buttEnt.isClick() ) {
        myTimer.setTimeout(_LED_OFF);
        vNalivR(bygDoza);
      }

      if (buttUp.isClick() ) {

        if (bygDoza < 4) {
          bygDoza++;
        }
        vPrintDoza(32, 36);
        myTimer.setTimeout(_LED_OFF);
      }
      if (buttDn.isClick()  ) {
        if (bygDoza > 0) {
          bygDoza--;
        }
        vPrintDoza(32, 36);
        myTimer.setTimeout(_LED_OFF);
      }
      if (buttDn.isHolded()) {
        bygStatus = 100;
        vSavebygRumRuch();
      }
      if (buttUp.isHolded()) {
        vSavebygRumRuch();
        bygStatus = 130;
      }
      break;
    case 130://**************************************************************** ручная калибровка
      vRuchLeds();
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      fPrintChar(21);// а
      bygPosN = 0;
      myTimer.setTimeout(_LED_OFF);
      bygStatus = 131;
      break;
    case 131://**************************************************************** ручная калибровка
      if (myTimer.isReady()) {
        vSavebygRumRuch();
        bygStatus = 120;//
      }
      if (buttEnt.isClick() ) {
        if ( bygRumRuch[bygPosN] == 1) {
          bygRumRuch[bygPosN] = 0;
          cascade[1].on(0, bygPosN + 1 ); //красный
          cascade[1].off(1, bygPosN + 1);//зеленый
          cascade[1].off(2, bygPosN + 1);// синий
        } else {
          bygRumRuch[bygPosN] = 1;
          cascade[1].off(0, bygPosN + 1 ); //красный
          cascade[1].on(1, bygPosN + 1);//зеленый
          cascade[1].off(2, bygPosN + 1);// синий
        }
      }
      if (buttDn.isClick() ) {
        if (bygPosN < 5) {
          bygPosN++;
        }
      }
      if (buttUp.isClick()  ) {

        if (bygPosN > 0) {
          bygPosN--;
        }
      }

      bPovorot(bygPosDeg[bygPosN]);

      if (buttDn.isHolded()) {
        bygStatus = 120;
        vSavebygRumRuch();
      }
      if (buttUp.isHolded()) {
        vSavebygRumRuch();
        bygStatus = 20;
      }
      break;
    case 140://**************************************************************** sleep
      myTimer.setTimeout(_POW_OFF);
      cascade.clear();
      vPrintDoza(27, 38);
      bygStatus = 141;
      break;
    case 141://**************************************************************** sleep
      if (myTimer.isReady()) {
        EEPROM.update(52, bygLastStatus);
        EEPROM.update(54, _EPP_CON);
        EEPROM.update(60, bygDoza);
        vSavebygRum();
        bygStatus = 80;
      }
      if (bygLastStatus != 120) {
        if ( bSensRums()) {
          bygStatus = bygLastStatus;
        }
      }
      if (buttEnt.isClick() ) {
        bygStatus = bygLastStatus;
      }
      if (buttDn.isClick() ) {
        bygStatus = bygLastStatus;
      }
      if (buttUp.isClick() ) {
        bygStatus = bygLastStatus;
      }
      break;
  }//switch (bygStatus)
}// loop
void fPrintChar(byte iNumChar) {
  for (byte i = 0; i < 8; i ++)
  {
    //0 - адрес, либо номер устройства на шине SPI
    //j - индекс массива байт с символом
    //i - текущий ряд на матрице    //CountDigits[i] - значение(byte) которым заполнится ряд
    cascade[0].setRow(i, CountDigits[ iNumChar][i]);
  }
}
void bPovorot(byte iUgol) {
  static byte iUgolLast;
  if (iUgol != iUgolLast) {
    bSensRums();
    servo.setTargetDeg(iUgol);
    while (!servo.tick()) {
#if(_UGOL_)
      Serial.println(servo.getCurrentDeg());
#endif
    }
    iUgolLast = iUgol;
  }
}
void vTuneFotosens() {
  //  cascade[1].clear();
  igUst[0] =  analogRead(1);
  igUst[1] =  analogRead(2);
  igUst[2] =  analogRead(3);
  igUst[3] =  analogRead(4);
  igUst[4] =  analogRead(5);
  igUst[5] =  analogRead(6);
#if(_DEBAG_)
  Serial.println( igUst[0]);
  Serial.println( igUst[1]);
  Serial.println( igUst[2]);
  Serial.println( igUst[3]);
  Serial.println( igUst[4]);
  Serial.println( igUst[5]);
  Serial.println("Уставки без рюмок");
#endif
}
bool bLedRum(byte iPos, bool bRefFresh) {
  if ((bygRumLast[iPos] != bygRum[iPos]) || bRefFresh) {
    switch (bygRum[iPos]) {
      case 0:
        cascade[1].on(0, iPos + 1 ); //красный
        cascade[1].off(1, iPos + 1);//зеленый
        cascade[1].off(2, iPos + 1);// синий
#if(_DEBAG_)
        Serial.println("Красный");
#endif
        break;
      case 1:
        cascade[1].off(0, iPos + 1 ); //красный
        cascade[1].on(1, iPos + 1);//зеленый
        cascade[1].off(2, iPos + 1);// синий
#if(_DEBAG_)
        Serial.println("зеленый");
#endif
        break;
      case 2:
        cascade[1].off(0, iPos + 1 ); //красный
        cascade[1].off(1, iPos + 1);//зеленый
        cascade[1].on(2, iPos + 1);// синий
#if(_DEBAG_)
        Serial.println("Синий");
#endif
        break;
    }
    bygRumLast[iPos] = bygRum[iPos];
    return 1;
  } else return 0;
}
void vFotoRum(byte iPos) {
  if ((igUst[iPos] - analogRead(iPos + 1) > igUstDif[iPos]) ) {
    if ( bygRum[iPos] != 2) {
      bygRum[iPos] = 1;
    }
  }
  else  bygRum[iPos] = 0;
}
bool bSensRums() {
  bool Rsult = 0;
  for (byte iPos = 0; iPos < 6; iPos++) {
    vFotoRum(iPos);
    Rsult = Rsult +  bLedRum(iPos, 0);
  }
  return Rsult;
}//bSensRums()
void vSensRumsRefresh() {
  bool Rsult;
  for (int iPos = 0; iPos < 6; iPos++) {
    vFotoRum(iPos);
    bLedRum(iPos, 1);
  }

}//bSensRums()
void vSaveFotoSens() {
  EEPROM.put(0, igUst[0]);
  EEPROM.put(2, igUst[1]);
  EEPROM.put(4, igUst[2]);
  EEPROM.put(6, igUst[3]);
  EEPROM.put(8, igUst[4]);
  EEPROM.put(10, igUst[5]);
  EEPROM.put(12, _EPP_CON);
}
void vSaveServoPos() {
  EEPROM.update(13, bygPosDeg[0]);
  EEPROM.update(15, bygPosDeg[1]);
  EEPROM.update(17, bygPosDeg[2]);
  EEPROM.update(19, bygPosDeg[3]);
  EEPROM.update(21, bygPosDeg[4]);
  EEPROM.update(23, bygPosDeg[5]);
  EEPROM.update(25, _EPP_CON);
}
void vSaveTimePomp() {
  EEPROM.put(26, ulTimePamp[0]);
  EEPROM.put(30, ulTimePamp[1]);
  EEPROM.put(34, ulTimePamp[2]);
  EEPROM.put(38, _EPP_CON);
}
void vSaveFotoSensR() {
  EEPROM.put(39, igUstDif[0]);
  EEPROM.put(41, igUstDif[1]);
  EEPROM.put(43, igUstDif[2]);
  EEPROM.put(45, igUstDif[3]);
  EEPROM.put(47, igUstDif[4]);
  EEPROM.put(49, igUstDif[5]);
  EEPROM.put(51, _EPP_CON);
}
void vSavebygRum() {
  EEPROM.update(61, bygRum[0]);
  EEPROM.update(62, bygRum[1]);
  EEPROM.update(63, bygRum[2]);
  EEPROM.update(64, bygRum[3]);
  EEPROM.update(65, bygRum[4]);
  EEPROM.update(66, bygRum[5]);
}
void vSavebygRumRuch() {
  EEPROM.update(67, bygRumRuch[0]);
  EEPROM.update(68, bygRumRuch[1]);
  EEPROM.update(69, bygRumRuch[2]);
  EEPROM.update(70, bygRumRuch[3]);
  EEPROM.update(71, bygRumRuch[4]);
  EEPROM.update(72, bygRumRuch[5]);
  EEPROM.update(73, _EPP_CON);
}
void vGetbygRumRuch() {
  EEPROM.get(67, bygRumRuch[0]);
  EEPROM.get(68, bygRumRuch[1]);
  EEPROM.get(69, bygRumRuch[2]);
  EEPROM.get(70, bygRumRuch[3]);
  EEPROM.get(71, bygRumRuch[4]);
  EEPROM.get(72, bygRumRuch[5]);
#if(_DEBAG_)
  Serial.println("Руч");
  Serial.println( bygRumRuch[0]);
  Serial.println( bygRumRuch[1]);
  Serial.println( bygRumRuch[2]);
  Serial.println( bygRumRuch[3]);
  Serial.println( bygRumRuch[4]);
  Serial.println( bygRumRuch[5]);
#endif
}
void vGetbygRum() {
  EEPROM.get(61, bygRum[0]);
  EEPROM.get(62, bygRum[1]);
  EEPROM.get(63, bygRum[2]);
  EEPROM.get(64, bygRum[3]);
  EEPROM.get(65, bygRum[4]);
  EEPROM.get(66, bygRum[5]);
#if(_DEBAG_)
  Serial.println("Рюмки");
  Serial.println( bygRum[0]);
  Serial.println( bygRum[1]);
  Serial.println( bygRum[2]);
  Serial.println( bygRum[3]);
  Serial.println( bygRum[4]);
  Serial.println( bygRum[5]);
#endif
}
void vGetTimePomp() {
  EEPROM.get(26, ulTimePamp[0]);
  EEPROM.get(30, ulTimePamp[1]);
  EEPROM.get(34, ulTimePamp[2]);
#if(_DEBAG_)
  Serial.println("Насос");
  Serial.println( ulTimePamp[0]);
  Serial.println( ulTimePamp[1]);
  Serial.println( ulTimePamp[2]);
#endif
}
void vGetFotoSens() {
  EEPROM.get(0, igUst[0]);
  EEPROM.get(2, igUst[1]);
  EEPROM.get(4, igUst[2]);
  EEPROM.get(6, igUst[3]);
  EEPROM.get(8, igUst[4]);
  EEPROM.get(10, igUst[5]);
#if(_DEBAG_)
  Serial.println("Фото");
  Serial.println( igUst[0]);
  Serial.println( igUst[1]);
  Serial.println( igUst[2]);
  Serial.println( igUst[3]);
  Serial.println( igUst[4]);
  Serial.println( igUst[5]);
#endif
}
void vGetServoPos() {
  EEPROM.get(13, bygPosDeg[0]);
  EEPROM.get(15, bygPosDeg[1]);
  EEPROM.get(17, bygPosDeg[2]);
  EEPROM.get(19, bygPosDeg[3]);
  EEPROM.get(21, bygPosDeg[4]);
  EEPROM.get(23, bygPosDeg[5]);
#if(_DEBAG_)
  Serial.println("Серво");
  Serial.println( bygPosDeg[0]);
  Serial.println( bygPosDeg[1]);
  Serial.println( bygPosDeg[2]);
  Serial.println( bygPosDeg[3]);
  Serial.println( bygPosDeg[4]);
  Serial.println( bygPosDeg[5]);
#endif
}
void vGetFotoSensR() {
  EEPROM.get(39, igUstDif[0]);
  EEPROM.get(41, igUstDif[1]);
  EEPROM.get(43, igUstDif[2]);
  EEPROM.get(45, igUstDif[3]);
  EEPROM.get(47, igUstDif[4]);
  EEPROM.get(49, igUstDif[5]);
#if(_DEBAG_)
  Serial.println("Диф");
  Serial.println( igUstDif[0]);
  Serial.println( igUstDif[1]);
  Serial.println( igUstDif[2]);
  Serial.println( igUstDif[3]);
  Serial.println( igUstDif[4]);
  Serial.println( igUstDif[5]);
#endif
}
void vCalibServo() {

  //  static bool flagLad;
  if (!servo.tick()) {
#if(_DEBAG_)
    Serial.println(servo.getCurrentDeg());
#endif
  }
  if (buttEnt.isHolded()) {
    vSaveServoPos();
    cascade[0].setRow(7, 255);
  }
  if (buttEnt.isClick()) {
    cascade[0].setRow(7, 0);
    if (bygPosN < 6) {
      cascade[1].clear();
      bygPosN++;
      cascade[1].on(1, (bygPosN + 1)); // зеленый
    }
    if (bygPosN > 5) {
      cascade[1].clear();
      bygPosN = 0;
      cascade[1].on(1, (bygPosN + 1)); // зеленый
    }
    //    servo.setTargetDeg(bygPosDeg[bygPosN]);

  }
  if (buttDn.isClick()) {
    if (bygPosDeg[bygPosN] < 180) {
      bygPosDeg[bygPosN]++;
    }
  }
  if (buttUp.isClick()) {
    if (bygPosDeg[bygPosN] > 0) {
      bygPosDeg[bygPosN]--;
    }
  }
  bPovorot(bygPosDeg[bygPosN]);
}
void vPumpCalib() {

  static unsigned long ulTimeNaliv = 0;
  static unsigned long ulStartTime;
  static unsigned long ulStopTime;
  static unsigned long ulDifTime = 0;
  static unsigned long ulStartNalivTime;

  static bool bTrigStart = 0;
  static byte iDoza;

  if (buttUp.isClick()) {
    if (bygPosN < 6) {
      bygPosN++;
      cascade[0].setRow(7, 0);
      cascade[0].on(7, bygPosN);
    }
    switch (bygPosN) {
      case 0:
        bPovorot(bygPosDeg[0]);
        break;
      case 1:
        bPovorot(bygPosDeg[1]);
        if (ulTimeNaliv != 0 ) {
          ulTimePamp[0] = ulTimeNaliv;
          bygRum[0] = 2;
          break;
        case 2:
          bPovorot(bygPosDeg[2]);
          break;
        case 3:
          bPovorot(bygPosDeg[3]);
          if ( ulTimeNaliv != 0 ) {
            ulTimePamp[1] = ulTimeNaliv;
            Serial.println(ulTimePamp[1]);
            bygRum[2] = 2;
          }
          break;
        case 4:
          bPovorot(bygPosDeg[4]);
          break;
        case 5:
          bPovorot(bygPosDeg[5]);
          if ( ulTimeNaliv != 0 ) {
            ulTimePamp[2] = ulTimeNaliv;
            bygRum[4] = 2;
          }
          break;
        case 6:
          cascade[0].setRow(7, 0);
          cascade[0].on(7, 7);
          break;
        }
    }
    ulTimeNaliv = 0;
  }
  if (buttDn.isClick()) {
    if (bygPosN > 0) {
      bygPosN--;
      cascade[0].setRow(7, 0);
      cascade[0].on(7, bygPosN);
    }
    switch (bygPosN) {
      case 0:
        bPovorot(bygPosDeg[0]);
        break;
      case 1:
        bPovorot(bygPosDeg[1]);
        break;
      case 2:
        bPovorot(bygPosDeg[2]);
        break;
      case 3:
        bPovorot(bygPosDeg[3]);
        break;
      case 4:
        bPovorot(bygPosDeg[4]);
        break;
      case 5:
        bPovorot(bygPosDeg[5]);
        break;
    }
  }
  if (buttEnt.state() and (bygPosN == 0 or bygPosN == 2 or bygPosN == 4 )) {
    if ( bygRum[bygPosN] == 1) {
      if (!bTrigStart) {
        bTrigStart = 1;
        ulStartTime = millis();
        digitalWrite(PIN_PUMP_ON, 1);
      }
    }
  } else {
    digitalWrite(PIN_PUMP_ON, 0);
    if (bTrigStart) {
      bTrigStart = 0;
      ulStopTime = millis();
      digitalWrite(PIN_PUMP_ON, 0);
      ulDifTime = ulStopTime - ulStartTime;
      ulTimeNaliv = ulTimeNaliv + ulDifTime;
#if(_DEBAG_)
      Serial.println(ulTimeNaliv);
#endif
      digitalWrite(PIN_PUMP_ON, 0);
    }
  }
  if (buttEnt.isClick() ) {
    if (bygPosN == 1 or bygPosN == 3 or bygPosN == 5 ) {
      switch (bygPosN) {
        case 1:
          iDoza = 0;
          break;
        case 3:
          iDoza = 1;
          break;
        case 5:
          iDoza = 2;
          break;
      }// end switch
      if ( bygRum[bygPosN] == 1) {
        vPump(ulTimePamp[iDoza]);
        bygRum[bygPosN] = 2;
        vSensRumsRefresh();
      }
    }
    if (bygPosN == 6 ) {
      vSaveTimePomp();
      cascade[0].setRow(7, 0);
    }
  }
}
void vPump(unsigned long ulTime) {
  if (ulTime > 0) {
    myTimer.setTimeout(ulTime);   // настроить таймаут
    digitalWrite(PIN_PUMP_ON, 1);
    while (!myTimer.isReady()) {
      bSensRums();
    }
    digitalWrite(PIN_PUMP_ON, 0);
  }
}
void  DelayWithSensRum(unsigned long ulTime) {
  myTimer.setTimeout(ulTime);   // настроить таймаут
  while (!myTimer.isReady()) {
    bSensRums();
  }
}
int  DelayWithSensRum( byte iStatCuret, byte StatJamp) {

  if (myTimer.isReady()) {
    return StatJamp;
  } else {
    return iStatCuret;
  }
}
void vNaliv(byte iDoz) {
  bygPosNaliv = 0;
  while (bygPosNaliv < 100) {
    bSensRums();
    switch (bygPosNaliv) {
      case 0:
        bygPosNaliv = 10;
        break;
      case 10:
        if ( bygRum[0] == 1) {// 1 рюмка
          bPovorot(bygPosDeg[0]);
          //          DelayWithSensRum(_DO_NALIV);
          switch (iDoz) {
            case 3:
              vPump(ulTimePamp[2]);
              break;
            case 4:
              vPump(ulTimePamp[2]);
              break;
            default:
              vPump(ulTimePamp[bygDoza]);
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[0] = 2;
        }
        bygPosNaliv = 20;
        break;
      case 20:
        if ( bygRum[1] == 1) {// 2 рюмка
          bPovorot(bygPosDeg[1]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRum[1] == 1) {
#if(_DEBAG_)
            Serial.println(iDoz);
#endif
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[2]);
                break;
              case 4:
                vPump(ulTimePamp[2]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[1] = 2;
        }
        bygPosNaliv = 30;
        break;
      case 30:
        if ( bygRum[2] == 1) {// 3 рюмка
          bPovorot(bygPosDeg[2]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRum[2] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[1]);
                break;
              case 4:
                vPump(ulTimePamp[2]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[2] = 2;
        }
        bygPosNaliv = 40;
        break;
      case 40:
        if ( bygRum[3] == 1) {// 4 рюмка
          bPovorot(bygPosDeg[3]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRum[3] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[1]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[3] = 2;
        }
        bygPosNaliv = 50;
        break;
      case 50:
        if ( bygRum[4] == 1) {// 5 рюмка
          bPovorot(bygPosDeg[4]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRum[4] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[0]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[4] = 2;
        }
        bygPosNaliv = 60;
        break;
      case 60:
        if ( bygRum[5] == 1) {// 6 рюмка
          bPovorot(bygPosDeg[5]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRum[5] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[0]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          bygRum[5] = 2;
        }
        bygPosNaliv = 70;
        break;
      case 70:
        bPovorot(bygPosDeg[0]);
        bygPosNaliv = 100;
        break;
    }
  }
}
void vPrintCapBat() {

  int capacity;
  int volts =  analogRead(7) * _ARDUINO_PIT / 1023 * 1000;
  if (volts > 3870)
    capacity = map(volts, 4200, 3870, 100, 77);
  else if ((volts <= 3870) && (volts > 3750) )
    capacity = map(volts, 3870, 3750, 77, 54);
  else if ((volts <= 3750) && (volts > 3680) )
    capacity = map(volts, 3750, 3680, 54, 31);
  else if ((volts <= 3680) && (volts > 3400) )
    capacity = map(volts, 3680, 3400, 31, 8);
  else if (volts <= 3400)
    capacity = map(volts, 3400, 2600, 8, 0);
#if(_PRINT_BAT_)
  Serial.println(volts);
  Serial.println(capacity);
#endif
  if (capacity <= 10) {
    fPrintChar(10);// батарейка 0 -10%
  }
  if ((10 < capacity) && (capacity <= 30) ) {
    fPrintChar(11);// батарейка 10 - 30%
  }
  if ((30 < capacity) && (capacity <= 50) ) {
    fPrintChar(12);// батарейка 30 - 50%
  }
  if ((50 < capacity) && (capacity <= 60)) {
    fPrintChar(13);// батарейка  50 - 60 %
  }
  if ((60 < capacity) && (capacity <= 80)) {
    fPrintChar(14);// батарейка  60 - 80%
  }
  if ((80 < capacity) && (capacity <= 90)) {
    fPrintChar(15);// батарейка 80% - 90%
  }
  if ( capacity > 90) {
    fPrintChar(16);// батарейка 100%
  }
}
bool vLedEf2(byte * LedData, bool bButton) {
  static bool bFlag;
  static byte iCount = 0;
  if (bButton) {
    bFlag = 1;
    iCount = 0;
    cascade[1].clear();
  }

  if (( (LedData[iCount] != 0) && myTimer.isReady()) || bFlag ) {
    myTimer.setTimeout(LedData[iCount] * 100); // настроить таймаут
    iCount++;
    cascade[1].setRow(0, LedData[iCount]);//red
    iCount++;
    cascade[1].setRow(1, LedData[iCount]);//green
    iCount++;
    cascade[1].setRow(2, LedData[iCount]);//blue
    iCount++;
    bFlag = 0;
    return 0;
  } else {
    if ( LedData[iCount] == 0) {
      return 1;
    } else return 0;
  }
}
byte LedEfNum(bool bPlus, bool bMinus) {
  static byte iNum;
  if (bPlus && (iNum < 9)) {
    iNum++;
    fPrintChar(iNum);
  }
  if (bMinus && (iNum > 0)) {
    iNum--;
    fPrintChar(iNum);
  }
  return iNum;
}
void vLedEfSel(int iSel) {
  switch (iSel) {
    case 1:
      vLedEf2(LedData1,  buttEnt.isSingle());
      break;
    case 2:
      vLedEf2(LedData2,  buttEnt.isSingle());
      break;
    case 3:
      vLedEf2(LedData3,  buttEnt.isSingle());
      break;
  }
}
void vRuchLeds() {
  for (byte iPosFor = 0; iPosFor < 6; iPosFor++) {
    if ( bygRumRuch[iPosFor] == 0) {
      cascade[1].on(0, iPosFor + 1 ); //красный
      cascade[1].off(1, iPosFor + 1);//зеленый
      cascade[1].off(2, iPosFor + 1);// синий
    } else {
      cascade[1].off(0, iPosFor + 1 ); //красный
      cascade[1].on(1, iPosFor + 1);//зеленый
      cascade[1].off(2, iPosFor + 1);// синий
    }
  }
}
void vNalivR(byte iDoz) {
  bygPosNaliv = 0;
  while (bygPosNaliv < 100) {
    bSensRums();
    switch (bygPosNaliv) {
      case 0:
        bygPosNaliv = 10;
        break;
      case 10:
        if ( bygRumRuch[0] == 1) {
          bPovorot(bygPosDeg[0]);
          //          DelayWithSensRum(_DO_NALIV);
          switch (iDoz) {
            case 3:
              vPump(ulTimePamp[2]);
              break;
            case 4:
              vPump(ulTimePamp[2]);
              break;
            default:
              vPump(ulTimePamp[bygDoza]);
          }
          DelayWithSensRum(_POSLE_NALIV);
          //   bygRumRuch[0] = 2;
        }
        bygPosNaliv = 20;
        break;
      case 20:
        if ( bygRumRuch[1] == 1) {
          bPovorot(bygPosDeg[1]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRumRuch[1] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[2]);
                break;
              case 4:
                vPump(ulTimePamp[2]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          bygRumRuch[1] = 2;
        }
        bygPosNaliv = 30;
        break;
      case 30:
        if ( bygRumRuch[2] == 1) {
          bPovorot(bygPosDeg[2]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRumRuch[2] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[1]);
                break;
              case 4:
                vPump(ulTimePamp[2]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //         bygRumRuch[2] = 2;
        }
        bygPosNaliv = 40;
        break;
      case 40:
        if ( bygRumRuch[3] == 1) {
          bPovorot(bygPosDeg[3]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRumRuch[3] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[1]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //         bygRumRuch[3] = 2;
        }
        bygPosNaliv = 50;
        break;
      case 50:
        if ( bygRumRuch[4] == 1) {
          bPovorot(bygPosDeg[4]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRumRuch[4] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[0]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          bygRumRuch[4] = 2;
        }
        bygPosNaliv = 60;
        break;
      case 60:
        if ( bygRumRuch[5] == 1) {
          bPovorot(bygPosDeg[5]);
          DelayWithSensRum(_DO_NALIV);
          if ( bygRumRuch[5] == 1) {
            switch (iDoz) {
              case 3:
                vPump(ulTimePamp[0]);
                break;
              case 4:
                vPump(ulTimePamp[1]);
                break;
              default:
                vPump(ulTimePamp[bygDoza]);
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          bygRumRuch[5] = 2;
        }
        bygPosNaliv = 70;
        break;
      case 70:
        bPovorot(bygPosDeg[0]);
        bygPosNaliv = 100;
        break;
    }
  }
}
void vTuneFotosensR() {
  igUstDif[0] = (igUst[0] -  analogRead(1)) / 4;
  igUstDif[1] =  (igUst[1] -  analogRead(2)) / 4;
  igUstDif[2] =  (igUst[2] -  analogRead(3)) / 4;
  igUstDif[3] =  (igUst[3] -  analogRead(4)) / 4;
  igUstDif[4] =  (igUst[4] -  analogRead(5)) / 4;
  igUstDif[5] =  (igUst[5] -  analogRead(6)) / 4;
#if(_DEBAG_)
  Serial.println( igUstDif[0]);
  Serial.println( igUstDif[1]);
  Serial.println( igUstDif[2]);
  Serial.println( igUstDif[3]);
  Serial.println( igUstDif[4]);
  Serial.println( igUstDif[5]);
  Serial.println("Уставки диф");
#endif
}
void vPrintDoza(byte bySdvig, byte bySdvig2) {
  fPrintChar(bygDoza + bySdvig);
  switch (bygDoza) {
    case 3:
      fPrintChar(bygDoza + bySdvig2);
      break;
    case 4:
      fPrintChar(bygDoza + bySdvig2);
      break;
    default:
      fPrintChar(bygDoza + bySdvig);
  }
}
