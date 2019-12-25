//08 12 2019
#define _DEBAG_ 0
#define _UGOL_ 0
#define _PRINT_BAT_ 0


#define _FOTO_DIF_1 100
#define _FOTO_DIF_2 100
#define _FOTO_DIF_3 100
#define _FOTO_DIF_4 100
#define _FOTO_DIF_5 100
#define _FOTO_DIF_6 100

#define _DEBOUNCE_R 10// настройка антидребезга (по умолчанию 80 мс)
#define _TIME_OUT_R 1000// настройка таймаута на удержание (по умолчанию 500 мс)
#define _CLICK_TIME_R 600// настройка таймаута между кликами (по умолчанию 300 мс)
#define _SPEED_SERVO_R 50//установка максимальной скорости (условные единицы, 0 – 200)
#define _ACCEL_SERVO_R 1//становка ускорения (0.05 – 1). При значении 1 ускорение максимальное
#define _DO_NALIV 100 //задержка до налива мс
#define _POSLE_NALIV 200 //задержка после налива мс
#define _LED_OFF 15000 //задержка на отключение светодиодов
#define _POW_OFF 15000 //задержка на отключение светодиодов


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
#include "simbols.h"
#include "MatrixCascade.h"
MatrixCascade<2> cascade(12, 11, 10);

#include "GyverTimer.h"
GTimer myTimer(MS);    // создать миллисекундный таймер

#include "ServoSmooth.h"
ServoSmooth servo;

#include <EEPROM.h>

int iUst[6] {972, 903, 755, 716, 574, 876};
int iUstDif[6] {_FOTO_DIF_1, _FOTO_DIF_2, _FOTO_DIF_3, _FOTO_DIF_4, _FOTO_DIF_5, _FOTO_DIF_6};
int igPosDeg[6] {0, 37, 63, 97, 129, 180};
int iRum[6];
int iRumRuch[6] {1, 0, 0, 0, 0, 0};
int iRumLast[6] {1, 1, 1, 1, 1, 1};
unsigned long ulTimePamp[3] = {100, 200, 300};
int igDoza = 2;
int iAutoDoza =  0;
int iPosN;
int iLastStatus = 20;
 byte byTemp;
 byte byTemp1;

void setup() {

  pinMode(PIN_SERVO_ON, OUTPUT);
  pinMode(PIN_PUMP_ON, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, 0);
  digitalWrite(PIN_PUMP_ON, 0);
  digitalWrite(PIN_SERVO_ON, 1);
#if(_DEBAG_)
#pragma message "D_E_B_A_G"
  Serial.begin(9600);
#endif
#if(_DEBAG_)
  Serial.println("D_E_B_A_G");
#endif
 

  EEPROM.get(12, byTemp);
  if (byTemp == 77) {
    vGetFotoSens();
  } else {
#if(_DEBAG_)
    Serial.println("eep Foto !");
#endif
  }
  EEPROM.get(51, byTemp);
  if (byTemp == 77) {
    vGetFotoSensR();
  } else {
#if(_DEBAG_)
    Serial.println("eep Diff !");
#endif
  }
  EEPROM.get(54, byTemp);
  if (byTemp == 77) {
    EEPROM.get(52, iLastStatus);
  } else {
#if(_DEBAG_)
    Serial.println("eep Stat !");
#endif
  }

  EEPROM.get(25, byTemp);
  if (byTemp == 77) {
    vGetServoPos();
  } else {
#if(_DEBAG_)
    Serial.println("eep servo !");
#endif
  }

  EEPROM.get(38, byTemp);
  if (byTemp == 77) {
    vGetTimePomp();
  } else {
#if(_DEBAG_)
    Serial.println("eep Pump !");
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

  vStatus();

  //  vTestButtonEnt();
  //  vTestButtonUp();
  //  vTestButtonDn();

  //vTestServo();
  //  vTestServo2();
}
void vStatus() {
  static int iStatus;
  switch (iStatus) {
    case 0:
     if (buttEnt.state()) {
        iStatus = 9;
       }else iStatus = 80;
      
       break;
      case 9:
      digitalWrite(5, 0);
       cascade.clear();
      vLedEf2(LedData2, 1);
      delay(1);
      vPrintCapBat();
      iStatus = 10;
       break;
      break;
    case 10:
      if (vLedEf2(LedData2, 0)) {
        iStatus = iLastStatus;
      }
      break;
    case 20://*************************************************************** 20 РЮМКА (Доза)
      fPrintChar(igDoza + 18);
      vSensRumsRefresh();
      iStatus = 21;
      buttDn.isHolded();
      buttUp.isHolded();
      buttEnt.isClick();
      myTimer.setTimeout(_LED_OFF);
      buttDn.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)
      bPovorot(igPosDeg[0]);
      break;
    case 21://
      if (myTimer.isReady()) {
        iStatus = 140;// на сон
        iLastStatus = 20;
      }
      if ( bSensRums()) {
        myTimer.setTimeout(_LED_OFF);
        fPrintChar(igDoza + 18);
        vSensRumsRefresh();
      }
      if (buttUp.isClick() ) {
        vSensRumsRefresh();
        myTimer.setTimeout(_LED_OFF);
        if (igDoza < 2) {
          igDoza++;
        }
        fPrintChar(igDoza + 18);
      }
      if (buttDn.isClick()  ) {
        vSensRumsRefresh();
        myTimer.setTimeout(_LED_OFF);
        if (igDoza > 0) {
          igDoza--;
        }
        fPrintChar(igDoza + 18);
      }
      if (buttDn.isHolded()) {
        iStatus = 70;//
      }
      if (buttUp.isHolded()) {
        iStatus = 100;// на автоналив
      }
      if (buttEnt.isClick()) {
        digitalWrite(5, 0);
        iStatus = 40;// на налив
        vSensRumsRefresh();
      }
      break;
    case 30://********************************************************************** 30  калибровка фтотодатчиков
      fPrintChar(17);//Ф
      iStatus = 31;
      buttEnt.isHolded();
      buttUp.isHolded();
      buttEnt.isClick();
      vSensRumsRefresh();
      buttDn.setTimeout(_TIME_OUT_R);        // настройка таймаута на удержание (по умолчанию 500 мс)
      bPovorot(igPosDeg[0]);
      break;
    case 31://----------------------------------------------------------------------31  калибровка фтотодатчиков
      bSensRums();
      if (buttEnt.isClick()) {
        vTuneFotosens();
        vSaveFotoSens();
        cascade[0].on(7, 0);
      }
      if (buttUp.isHolded()) {
        iStatus = 70;
      }
      if (buttDn.isHolded()) {
        iStatus = 60;
      }
      if (buttEnt.isHolded()) {
        vTuneFotosensR();
        vSaveFotoSensR();
        cascade[0].off(7, 0);
      }
      break;
    case 40:// ****************************************************** 40 налив
      vPrintCapBat();
      vNaliv(0);
#if(_DEBAG_)
      Serial.println("Налито");
#endif
      iStatus = 20;// на рюмку
      break;

    case 50://******************************************************** 50 калибровка серво
      cascade.clear();
      fPrintChar(22);//c
      cascade[1].on(1, (1)); // зеленый
      iStatus = 51;
      buttUp.isHolded();
      buttDn.isHolded();
      buttEnt.isHolded();
      buttEnt.isClick();
      buttDn.isClick();
      buttUp.isClick();
      cascade[1].clear();
      iPosN = 0;
      cascade[1].on(1, (iPosN + 1)); // зеленый
      bPovorot(igPosDeg[0]);
      break;
    case 51:// -----------------------------------------------------------51 калиб. серво
      vCalibServo();
      if (buttUp.isHolded()) {
        bPovorot(igPosDeg[0]);
        iStatus = 60;// на калиб. насос
      }
      if (buttDn.isHolded()) {
        bPovorot(igPosDeg[0]);
        iStatus = 20;// на рюмку
      }
      break;
    case 60://************************************************************ 60 Калибровка насоса
      fPrintChar(23);// н
      cascade[0].on(7, 0);
      buttEnt.setDebounce(10);// настройка антидребезга
      iStatus = 61;
      buttUp.isHolded();
      buttUp.isClick();
      buttDn.isClick();
      buttEnt.isClick();
      vSensRumsRefresh();
      iPosN = 0;
      bPovorot(igPosDeg[0]);
      break;
    case 61://Калибровка насоса
      vPumpCalib();
      if (buttUp.isHolded()) {
        buttEnt.setDebounce(_DEBOUNCE_R);// настройка антидребезга
        iStatus = 30;// на фотодатчики
      }
      if (buttDn.isHolded()) {
        buttEnt.setDebounce(_DEBOUNCE_R);// настройка антидребезга
        iStatus = 50;// на серво
      }
      break;
    case 70://************************************************************** 70 Промывка
      vSensRumsRefresh();
      vPrintCapBat();
      iStatus = 71;
      buttDn.isHolded();
      buttDn.isClick();
      buttUp.isClick();
      buttUp.isHolded();
      myTimer.setTimeout(5000);   // настроить таймаут
      buttDn.setTimeout(3000);        // настройка таймаута на удержание (по умолчанию 500 мс)
      break;
    case 71://Промывка
      bSensRums();
      iStatus = DelayWithSensRum(iStatus, 20);
      if (buttUp.isClick() or buttUp.isHolded()) {
        iStatus = 20;// на рюмку
      }
      if (buttDn.isHolded()) {
        buttEnt.isClick();
        myTimer.setTimeout(5000);   // настроить таймаут
        iStatus = 30;// на калиб фото
      }
      if (buttEnt.state()) {
        if ( iRum[0] != 0) {
          digitalWrite(PIN_PUMP_ON, 1);
          iRum[0] = 2;
        }
        myTimer.setTimeout(5000);   // настроить таймаут
      } else digitalWrite(PIN_PUMP_ON, 0);
      break;
    case 80://Зарядка ******************************************************* 80 Зарядка
      cascade.clear();
      iStatus = 81;
      vPrintCapBat();//   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      digitalWrite(5, 1);
       myTimer.setInterval(300);
       byTemp = 0;
       byTemp1 = 0;
     
      break;
    case 81://Зарядка ******************************************************* 81 Зарядка

       if (buttEnt.state()) {
        iStatus = 9;
       }
if (myTimer.isReady() ){
          byTemp ++;
          if (byTemp == 1){
           cascade[0].setRow(0, B10111101);
          }
          if (byTemp == 2){
            cascade[0].setRow(0, B00111100);
          }
          if (byTemp > 2){
            byTemp = 0;
             byTemp1++;
          }
          if (byTemp1 == 9){  
      digitalWrite(5, 0); 
          }
          if (byTemp1 > 10){
            byTemp1 = 0;
            vPrintCapBat();//   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      digitalWrite(5, 1); 
          }
          
        }
      
      break;
    case 90://****************************************************************90 Световые эфекты
      buttUp.isClick();
      buttDn.isClick();
      fPrintChar(LedEfNum(buttUp.isClick(), buttDn.isClick()));// 0 - 9
      iStatus = 91;
      buttDn.isHolded();
      buttEnt.isClick();
      break;
    case 91://*****************************************************************91 Световые эфекты
      int iNumEf ;
      if (buttDn.isHolded()) {
        iStatus = 130;
      }
      if (buttUp.isHolded()) {
        iStatus = 20;
      }
      LedEfNum(buttUp.isClick(), buttDn.isClick());
      if  ( buttEnt.isClick()) {
        iNumEf = LedEfNum(buttUp.isClick(), buttDn.isClick());
      }
      vLedEfSel(iNumEf);
      break;
    case 100://**************************************************************** автоналив
      fPrintChar(igDoza + 24);// а
      iStatus = 101;
      vSensRumsRefresh();
      myTimer.setTimeout(_LED_OFF);
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      break;
    case 101://***************************************************************************** автоналив
      if (myTimer.isReady()) {
        iLastStatus = 100;
        iStatus = 140;// на сон
      }
      if (bSensRums()) {
        iStatus = 102;
      }

      if (buttEnt.isClick() ) {
        vPrintCapBat();
        vNaliv(0);
        fPrintChar(igDoza + 24);// а
      }
      if (buttUp.isClick() ) {
        if (igDoza < 2) {
          igDoza++;
          fPrintChar(igDoza + 24);

        }
      }
      if (buttDn.isClick()  ) {
        if (igDoza > 0) {
          igDoza--;
          fPrintChar(igDoza + 24);

        }
      }

      if (buttDn.isHolded()) {
        iStatus = 20;
      }
      if (buttUp.isHolded()) {
        iStatus = 110;
      }
      break;
    case 102:
      if (bSensRums()) {
        myTimer.setTimeout(3000);
      }
      if (myTimer.isReady()) {
        vPrintCapBat();
        vNaliv(0);
        iStatus = 100;
      }
      break;
    case 110://**************************************************************** 2 - 3 дозы
      fPrintChar(30 + iAutoDoza);
      iStatus = 111;
      vSensRumsRefresh();
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      break;
    case 111:// автоналив
      if (bSensRums()) {
        myTimer.setTimeout(3000);   // настроить таймаут
      }
      if (myTimer.isReady()) {
        vPrintCapBat();
        vNaliv(iAutoDoza + 1);
        fPrintChar(30 + iAutoDoza);// а
      }
      if (buttEnt.isClick() ) {
        vPrintCapBat();
        vNaliv(iAutoDoza + 1);
        fPrintChar(30 +  iAutoDoza);// а
      }
      if (buttUp.isClick() ) {
        if (iAutoDoza < 1) {
          iAutoDoza = 1;
        }
        fPrintChar(30 + iAutoDoza);
      }
      if (buttDn.isClick()  ) {
        if (iAutoDoza > 0) {
          iAutoDoza = 0;
        }
        fPrintChar(30 + iAutoDoza);
      }

      if (buttDn.isHolded()) {
        iStatus = 100;
      }
      if (buttUp.isHolded()) {
        iStatus = 120;
      }
      break;
    case 120://**************************************************************** ручной налив
      vRuchLeds();
      bPovorot(igPosDeg[0]);
      fPrintChar(32 + igDoza);// а
      iStatus = 121;
      buttEnt.isClick();
      buttUp.isClick();
      buttDn.isClick();
      buttDn.isClick();
      buttDn.isHolded();
      buttUp.isHolded();
      vRuchLeds();
      break;
    case 121://**************************************************************** ручной налив
      if (buttEnt.isClick() ) {
        vNalivR(0);
      }

      if (buttUp.isClick() ) {

        if (igDoza < 2) {
          igDoza++;
        }
        fPrintChar(igDoza + 32);
      }
      if (buttDn.isClick()  ) {
        if (igDoza > 0) {
          igDoza--;
        }
        fPrintChar(igDoza + 32);
      }
      if (buttDn.isHolded()) {
        iStatus = 110;
      }
      if (buttUp.isHolded()) {
        iStatus = 130;
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
      iStatus = 131;
      iRumRuch[0];
      iPosN = 0;
      break;
    case 131://**************************************************************** ручная калибровка
      vRuchRum();
      if (buttDn.isHolded()) {
        iStatus = 120;
      }
      if (buttUp.isHolded()) {
        iStatus = 90;
      }
      break;
    case 140://**************************************************************** sleep
      myTimer.setTimeout(_POW_OFF);
      cascade.clear();
      fPrintChar(27 + igDoza);
      iStatus = 141;
      break;
    case 141://**************************************************************** sleep
      if (myTimer.isReady()) {
        EEPROM.put(52, iLastStatus);
        EEPROM.put(54, 77);
       iStatus = 80;
      }
      if ( bSensRums()) {
        iStatus = iLastStatus;
      }
      if (buttEnt.isClick() ) {
        iStatus = iLastStatus;
      }
      if (buttDn.isClick() ) {
        iStatus = iLastStatus;
      }
      if (buttUp.isClick() ) {
        iStatus = iLastStatus;
      }
      break;
  }//switch (iStatus)
}//void vStatus()
void fPrintChar(int iNumChar) {
  for (int i = 0; i < 8; i ++)
  {
    //0 - адрес, либо номер устройства на шине SPI
    //j - индекс массива байт с символом
    //i - текущий ряд на матрице    //CountDigits[i] - значение(byte) которым заполнится ряд
    cascade[0].setRow(i, CountDigits[ iNumChar][i]);
  }
}
void bPovorot(int iUgol) {
  static int iUgolLast;
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
  iUst[0] =  analogRead(1);
  iUst[1] =  analogRead(2);
  iUst[2] =  analogRead(3);
  iUst[3] =  analogRead(4);
  iUst[4] =  analogRead(5);
  iUst[5] =  analogRead(6);
#if(_DEBAG_)
  Serial.println( iUst[0]);
  Serial.println( iUst[1]);
  Serial.println( iUst[2]);
  Serial.println( iUst[3]);
  Serial.println( iUst[4]);
  Serial.println( iUst[5]);
  Serial.println("Уставки без рюмок");
#endif
}
bool bLedRum(int iPos, bool bRefFresh) {
  if ((iRumLast[iPos] != iRum[iPos]) || bRefFresh) {
    switch (iRum[iPos]) {
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
    iRumLast[iPos] = iRum[iPos];
    return 1;
  } else return 0;
}
void vFotoRum(int iPos) {
  if ((iUst[iPos] - analogRead(iPos + 1) > iUstDif[iPos]) ) {
    if ( iRum[iPos] != 2) {
      iRum[iPos] = 1;
    }
  }
  else  iRum[iPos] = 0;
}
bool bSensRums() {
  bool Rsult;
  for (int iPos = 0; iPos < 6; iPos++) {
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
  EEPROM.put(0, iUst[0]);
  EEPROM.put(2, iUst[1]);
  EEPROM.put(4, iUst[2]);
  EEPROM.put(6, iUst[3]);
  EEPROM.put(8, iUst[4]);
  EEPROM.put(10, iUst[5]);
  EEPROM.put(12, 77);
}
void vSaveServoPos() {
  EEPROM.put(13, igPosDeg[0]);
  EEPROM.put(15, igPosDeg[1]);
  EEPROM.put(17, igPosDeg[2]);
  EEPROM.put(19, igPosDeg[3]);
  EEPROM.put(21, igPosDeg[4]);
  EEPROM.put(23, igPosDeg[5]);
  EEPROM.put(25, 77);
}
void vSaveTimePomp() {
  EEPROM.put(26, ulTimePamp[0]);
  EEPROM.put(30, ulTimePamp[1]);
  EEPROM.put(34, ulTimePamp[2]);
  EEPROM.put(38, 77);
}
void vSaveFotoSensR() {
  EEPROM.put(39, iUstDif[0]);
  EEPROM.put(41, iUstDif[1]);
  EEPROM.put(43, iUstDif[2]);
  EEPROM.put(45, iUstDif[3]);
  EEPROM.put(47, iUstDif[4]);
  EEPROM.put(49, iUstDif[5]);
  EEPROM.put(51, 77);
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
  EEPROM.get(0, iUst[0]);
  EEPROM.get(2, iUst[1]);
  EEPROM.get(4, iUst[2]);
  EEPROM.get(6, iUst[3]);
  EEPROM.get(8, iUst[4]);
  EEPROM.get(10, iUst[5]);
#if(_DEBAG_)
  Serial.println("Фото");
  Serial.println( iUst[0]);
  Serial.println( iUst[1]);
  Serial.println( iUst[2]);
  Serial.println( iUst[3]);
  Serial.println( iUst[4]);
  Serial.println( iUst[5]);

#endif
}
void vGetServoPos() {
  EEPROM.get(13, igPosDeg[0]);
  EEPROM.get(15, igPosDeg[1]);
  EEPROM.get(17, igPosDeg[2]);
  EEPROM.get(19, igPosDeg[3]);
  EEPROM.get(21, igPosDeg[4]);
  EEPROM.get(23, igPosDeg[5]);
#if(_DEBAG_)
  Serial.println("Серво");
  Serial.println( igPosDeg[0]);
  Serial.println( igPosDeg[1]);
  Serial.println( igPosDeg[2]);
  Serial.println( igPosDeg[3]);
  Serial.println( igPosDeg[4]);
  Serial.println( igPosDeg[5]);
#endif
}
void vGetFotoSensR() {
  EEPROM.get(39, iUstDif[0]);
  EEPROM.get(41, iUstDif[1]);
  EEPROM.get(43, iUstDif[2]);
  EEPROM.get(45, iUstDif[3]);
  EEPROM.get(47, iUstDif[4]);
  EEPROM.get(49, iUstDif[5]);
#if(_DEBAG_)
  Serial.println("Диф");
  Serial.println( iUstDif[0]);
  Serial.println( iUstDif[1]);
  Serial.println( iUstDif[2]);
  Serial.println( iUstDif[3]);
  Serial.println( iUstDif[4]);
  Serial.println( iUstDif[5]);
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
    if (iPosN < 6) {
      cascade[1].clear();
      iPosN++;
      cascade[1].on(1, (iPosN + 1)); // зеленый
    }
    if (iPosN > 5) {
      cascade[1].clear();
      iPosN = 0;
      cascade[1].on(1, (iPosN + 1)); // зеленый
    }
    //    servo.setTargetDeg(igPosDeg[iPosN]);

  }
  if (buttDn.isClick()) {
    if (igPosDeg[iPosN] < 180) {
      igPosDeg[iPosN]++;
    }
  }
  if (buttUp.isClick()) {
    if (igPosDeg[iPosN] > 0) {
      igPosDeg[iPosN]--;
    }
  }
  bPovorot(igPosDeg[iPosN]);
}
void vPumpCalib() {

  static unsigned long ulTimeNaliv = 0;
  static unsigned long ulStartTime;
  static unsigned long ulStopTime;
  static unsigned long ulDifTime = 0;
  static unsigned long ulStartNalivTime;

  static bool bTrigStart = 0;
  static int iDoza;
  vSensRumsRefresh();
  if (buttUp.isClick()) {
    if (iPosN < 6) {
      iPosN++;
      cascade[0].setRow(7, 0);
      cascade[0].on(7, iPosN);
    }
    switch (iPosN) {
      case 0:
        bPovorot(igPosDeg[0]);
        break;
      case 1:
        bPovorot(igPosDeg[1]);
        if (ulTimeNaliv != 0 ) {
          ulTimePamp[0] = ulTimeNaliv;
          iRum[0] = 2;
          break;
        case 2:
          bPovorot(igPosDeg[2]);
          break;
        case 3:
          bPovorot(igPosDeg[3]);
          if ( ulTimeNaliv != 0 ) {
            ulTimePamp[1] = ulTimeNaliv;
            Serial.println(ulTimePamp[1]);
            iRum[2] = 2;
          }
          break;
        case 4:
          bPovorot(igPosDeg[4]);
          break;
        case 5:
          bPovorot(igPosDeg[5]);
          if ( ulTimeNaliv != 0 ) {
            ulTimePamp[2] = ulTimeNaliv;
            iRum[4] = 2;
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
    if (iPosN > 0) {
      iPosN--;
      cascade[0].setRow(7, 0);
      cascade[0].on(7, iPosN);
    }
    switch (iPosN) {
      case 0:
        bPovorot(igPosDeg[0]);
        break;
      case 1:
        bPovorot(igPosDeg[1]);
        break;
      case 2:
        bPovorot(igPosDeg[2]);
        break;
      case 3:
        bPovorot(igPosDeg[3]);
        break;
      case 4:
        bPovorot(igPosDeg[4]);
        break;
      case 5:
        bPovorot(igPosDeg[5]);
        break;
    }
  }
  if (buttEnt.state() and (iPosN == 0 or iPosN == 2 or iPosN == 4 )) {
    if ( iRum[iPosN] == 1) {
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
    if (iPosN == 1 or iPosN == 3 or iPosN == 5 ) {
      switch (iPosN) {
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
      if ( iRum[iPosN] == 1) {
        vPump(ulTimePamp[iDoza]);
        iRum[iPosN] = 2;
      }
    }
    if (iPosN == 6 ) {
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
int  DelayWithSensRum( int iStatCuret, int StatJamp) {

  if (myTimer.isReady()) {
    return StatJamp;
  } else {
    return iStatCuret;
  }
}
void vNaliv(int iDoz) {
  int iPosNaliv = 0;
  while (iPosNaliv < 100) {
    bSensRums();
    switch (iPosNaliv) {
      case 0:
        iPosNaliv = 10;
        break;
      case 10:
        if ( iRum[0] == 1) {
          bPovorot(igPosDeg[0]);
          //          DelayWithSensRum(_DO_NALIV);
          switch (iDoz) {
            case 0:
              vPump(ulTimePamp[igDoza]);
              break;
            case 1:
              vPump(ulTimePamp[2]);
              break;
            case 2:
              vPump(ulTimePamp[2]);
              break;
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[0] = 2;
        }
        iPosNaliv = 20;
        break;
      case 20:
        if ( iRum[1] == 1) {
          bPovorot(igPosDeg[1]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRum[1] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[2]);
                break;
              case 2:
                vPump(ulTimePamp[2]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[1] = 2;
        }
        iPosNaliv = 30;
        break;
      case 30:
        if ( iRum[2] == 1) {
          bPovorot(igPosDeg[2]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRum[2] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[1]);
                break;
              case 2:
                vPump(ulTimePamp[2]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[2] = 2;
        }
        iPosNaliv = 40;
        break;
      case 40:
        if ( iRum[3] == 1) {
          bPovorot(igPosDeg[3]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRum[3] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[1]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[3] = 2;
        }
        iPosNaliv = 50;
        break;
      case 50:
        if ( iRum[4] == 1) {
          bPovorot(igPosDeg[4]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRum[4] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[0]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[4] = 2;
        }
        iPosNaliv = 60;
        break;
      case 60:
        if ( iRum[5] == 1) {
          bPovorot(igPosDeg[5]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRum[5] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[0]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          iRum[5] = 2;
        }
        iPosNaliv = 70;
        break;
      case 70:
        bPovorot(igPosDeg[0]);
        iPosNaliv = 100;
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
#if(_DEBAG_)
void vTestFoto() {
  int iUstRum[6];
  iUstRum[0] =  analogRead(1);
  iUstRum[1] =  analogRead(2);
  iUstRum[2] =  analogRead(3);
  iUstRum[3] =  analogRead(4);
  iUstRum[4] =  analogRead(5);
  iUstRum[5] =  analogRead(6);
  Serial.println( iUst[0]);
  Serial.println( iUst[1]);
  Serial.println( iUst[2]);
  Serial.println( iUst[3]);
  Serial.println( iUst[4]);
  Serial.println( iUst[5]);
  Serial.println("Уставки");
  Serial.println( iUstRum[0]);
  Serial.println( iUstRum[1]);
  Serial.println( iUstRum[2]);
  Serial.println( iUstRum[3]);
  Serial.println( iUstRum[4]);
  Serial.println( iUstRum[5]);
  Serial.println("Факт");

  Serial.println( iUst[0] - iUstRum[0]);
  Serial.println( iUst[1] - iUstRum[1]);
  Serial.println( iUst[2] - iUstRum[2]);
  Serial.println( iUst[3] - iUstRum[3]);
  Serial.println( iUst[4] - iUstRum[4]);
  Serial.println( iUst[5] - iUstRum[5]);
  Serial.println("Разница");
}
void vTestButtonEnt() {
  static int value;

  if (buttEnt.isClick()) Serial.println("Click Ent");         // проверка на один клик
  if (buttEnt.isSingle()) Serial.println("Single Ent");       // проверка на один клик
  if (buttEnt.isDouble()) Serial.println("Double Ent");       // проверка на двойной клик
  if (buttEnt.isTriple()) Serial.println("Triple Ent");       // проверка на тройной клик

  if (buttEnt.hasClicks())                                // проверка на наличие нажатий
    Serial.println(buttEnt.getClicks());                  // получить (и вывести) число нажатий

  if (buttEnt.isPress()) Serial.println("Press Ent");         // нажатие на кнопку (+ дебаунс)
  if (buttEnt.isRelease()) Serial.println("Release Ent");     // отпускание кнопки (+ дебаунс)
  if (buttEnt.isHolded()) Serial.println("Holded Ent");       // проверка на удержание
  if (buttEnt.isHold()) Serial.println("Holding Ent");        // проверка на удержание
  //if (buttEnt.state()) Serial.println("Hold Ent");          // возвращает состояние кнопки

  if (buttEnt.isStep()) {                                 // если кнопка была удержана (это для инкремента)
    value++;                                            // увеличивать/уменьшать переменную value с шагом и интервалом
    Serial.println(value);                              // для примера выведем в порт
  }
}
void vTestButtonUp() {
  static int value;

  if (buttUp.isClick()) Serial.println("Click Up");         // проверка на один клик
  if (buttUp.isSingle()) Serial.println("Single Up");       // проверка на один клик
  if (buttUp.isDouble()) Serial.println("Double Up");       // проверка на двойной клик
  if (buttUp.isTriple()) Serial.println("Triple Up");       // проверка на тройной клик

  if (buttUp.hasClicks())                                // проверка на наличие нажатий
    Serial.println(buttUp.getClicks());                  // получить (и вывести) число нажатий

  if (buttUp.isPress()) Serial.println("Press Up");         // нажатие на кнопку (+ дебаунс)
  if (buttUp.isRelease()) Serial.println("Release Up");     // отпускание кнопки (+ дебаунс)
  if (buttUp.isHolded()) Serial.println("Holded Up");       // проверка на удержание
  if (buttUp.isHold()) Serial.println("Holding Up");        // проверка на удержание
  //if (buttUp.state()) Serial.println("Hold Up");          // возвращает состояние кнопки

  if (buttUp.isStep()) {                                 // если кнопка была удержана (это для инкремента)
    value++;                                            // увеличивать/уменьшать переменную value с шагом и интервалом
    Serial.println(value);                              // для примера выведем в порт
  }
}
void vTestButtonDn() {
  static int value;

  if (buttDn.isClick()) Serial.println("Click Dn");         // проверка на один клик
  if (buttDn.isSingle()) Serial.println("Single Dn");       // проверка на один клик
  if (buttDn.isDouble()) Serial.println("Double Dn");       // проверка на двойной клик
  if (buttDn.isTriple()) Serial.println("Triple Dn");       // проверка на тройной клик

  if (buttDn.hasClicks())                                // проверка на наличие нажатий
    Serial.println(buttDn.getClicks());                  // получить (и вывести) число нажатий

  if (buttDn.isPress()) Serial.println("Press Dn");         // нажатие на кнопку (+ дебаунс)
  if (buttDn.isRelease()) Serial.println("Release Dn");     // отпускание кнопки (+ дебаунс)
  if (buttDn.isHolded()) Serial.println("Holded Dn");       // проверка на удержание
  if (buttDn.isHold()) Serial.println("Holding Dn");        // проверка на удержание
  //if (buttDn.state()) Serial.println("Hold Dn");          // возвращает состояние кнопки

  if (buttDn.isStep()) {                                 // если кнопка была удержана (это для инкремента)
    value++;                                            // увеличивать/уменьшать переменную value с шагом и интервалом
    Serial.println(value);                              // для примера выведем в порт
  }
}
void vTestServo() {
  static int iNewPos;
  if (buttUp.isClick()) {
    iNewPos = 30;
#if(_DEBAG_)
    Serial.println("=");
    Serial.println(iNewPos);
    Serial.println("=");

#endif
    servo.setTargetDeg(iNewPos);
  }
  if (buttDn.isClick()) {
    iNewPos = 10;
#if(_DEBAG_)
    Serial.println("=");
    Serial.println(iNewPos);
    Serial.println("=");

#endif
    servo.setTargetDeg(iNewPos);
  }
  if (buttEnt.isClick()) {
    iNewPos = 20;
#if(_DEBAG_)
    Serial.println("=");
    Serial.println(iNewPos);
    Serial.println("=");

#endif
    servo.setTargetDeg(iNewPos);
  }
  if (!servo.tick()) {
#if(_DEBAG_)
    Serial.println(servo.getCurrentDeg());
#endif
  }
  if (servo.getCurrentDeg() == 179) {
    cascade[1].on(1, 6);// зеленый
    cascade[1].off(1, 1);// зеленый
  }
  if (servo.getCurrentDeg() == 1) {
    cascade[1].on(1, 1);// зеленый
    cascade[1].off(1, 6);// зеленый
  }
}
void vTestServo2() {
  static int iPosN;
  static bool flagLad;
  static int iPosDeg[6] {0, 32, 72, 128, 154, 180};
  if (!servo.tick()) {
#if(_DEBAG_)
    Serial.println(servo.getCurrentDeg());
#endif
  }
  if (!flagLad) {
    flagLad = true;
    cascade[1].on(1, (iPosN + 1)); // зеленый
  }
  if (buttEnt.isClick()) {
    //    servo.setTargetDeg(iPosDeg[iPosN]);
    bPovorot(iPosDeg[iPosN]);
  }
  if (buttUp.isClick()) {
    if (iPosN < 5) {
      cascade[1].off(1, (iPosN + 1)); // зеленый
      iPosN++;
      cascade[1].on(1, (iPosN + 1)); // зеленый

#if(_DEBAG_)
      Serial.println(iPosN);
#endif
    }
  }
  if (buttDn.isClick()) {
    if (iPosN > 0) {
      cascade[1].off(1, (iPosN + 1)); // зеленый
      iPosN--;
      cascade[1].on(1, (iPosN + 1)); // зеленый
#if(_DEBAG_)
      Serial.println(iPosN);
#endif
    }
  }
}
void vLedEf(byte* LedData) {
  int iCount = 0;
  cascade[1].clear();
  while ( LedData[iCount] != 0) {
    myTimer.setTimeout(LedData[iCount] * 100); // настроить таймаут 3 сек
    iCount++;
    cascade[1].setRow(0, LedData[iCount]);//red
    iCount++;
    cascade[1].setRow(1, LedData[iCount]);//green
    iCount++;
    cascade[1].setRow(2, LedData[iCount]);//blue
    iCount++;
    while (!myTimer.isReady()) {
    }
  }
}
#endif

bool vLedEf2(byte* LedData, bool bButton) {
  static bool bFlag;
  static int iCount = 0;
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
int LedEfNum(bool bPlus, bool bMinus) {
  static int iNum;
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
void vRuchRum() {
  if (buttEnt.isClick() ) {
    if ( iRumRuch[iPosN] == 1) {
      iRumRuch[iPosN] = 0;
      cascade[1].on(0, iPosN + 1 ); //красный
      cascade[1].off(1, iPosN + 1);//зеленый
      cascade[1].off(2, iPosN + 1);// синий
    } else {
      iRumRuch[iPosN] = 1;
      cascade[1].off(0, iPosN + 1 ); //красный
      cascade[1].on(1, iPosN + 1);//зеленый
      cascade[1].off(2, iPosN + 1);// синий
    }
  }
  iRumRuch[iPosN];
  if (buttDn.isClick() ) {
    if (iPosN < 5) {
      iPosN++;
    }
  }
  if (buttUp.isClick()  ) {

    if (iPosN > 0) {
      iPosN--;
    }
  }

  bPovorot(igPosDeg[iPosN]);
}
void vRuchLeds() {
  for (int iPosFor = 0; iPosFor < 6; iPosFor++) {
    if ( iRumRuch[iPosFor] == 0) {
      cascade[1].on(0, iPosFor + 1 ); //красный
      cascade[1].off(1, iPosFor + 1);//зеленый
      cascade[1].off(2, iPosFor + 1);// синий
    } else {
      cascade[1].off(0, iPosFor + 1 ); //красный
      cascade[1].on(1, iPosFor + 1);//зеленый
      cascade[1].off(2, iPosFor + 1);// синий
    }
  }
} void vNalivR(int iDoz) {
  int iPosNaliv = 0;
  while (iPosNaliv < 100) {
    bSensRums();
    switch (iPosNaliv) {
      case 0:
        iPosNaliv = 10;
        break;
      case 10:
        if ( iRumRuch[0] == 1) {
          bPovorot(igPosDeg[0]);
          //          DelayWithSensRum(_DO_NALIV);
          switch (iDoz) {
            case 0:
              vPump(ulTimePamp[igDoza]);
              break;
            case 1:
              vPump(ulTimePamp[2]);
              break;
            case 2:
              vPump(ulTimePamp[2]);
              break;
          }
          DelayWithSensRum(_POSLE_NALIV);
          //   iRumRuch[0] = 2;
        }
        iPosNaliv = 20;
        break;
      case 20:
        if ( iRumRuch[1] == 1) {
          bPovorot(igPosDeg[1]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRumRuch[1] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[2]);
                break;
              case 2:
                vPump(ulTimePamp[2]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          iRumRuch[1] = 2;
        }
        iPosNaliv = 30;
        break;
      case 30:
        if ( iRumRuch[2] == 1) {
          bPovorot(igPosDeg[2]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRumRuch[2] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[1]);
                break;
              case 2:
                vPump(ulTimePamp[2]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //         iRumRuch[2] = 2;
        }
        iPosNaliv = 40;
        break;
      case 40:
        if ( iRumRuch[3] == 1) {
          bPovorot(igPosDeg[3]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRumRuch[3] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[1]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //         iRumRuch[3] = 2;
        }
        iPosNaliv = 50;
        break;
      case 50:
        if ( iRumRuch[4] == 1) {
          bPovorot(igPosDeg[4]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRumRuch[4] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[0]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          iRumRuch[4] = 2;
        }
        iPosNaliv = 60;
        break;
      case 60:
        if ( iRumRuch[5] == 1) {
          bPovorot(igPosDeg[5]);
          DelayWithSensRum(_DO_NALIV);
          if ( iRumRuch[5] == 1) {
            switch (iDoz) {
              case 0:
                vPump(ulTimePamp[igDoza]);
                break;
              case 1:
                vPump(ulTimePamp[0]);
                break;
              case 2:
                vPump(ulTimePamp[1]);
                break;
            }
          }
          DelayWithSensRum(_POSLE_NALIV);
          //          iRumRuch[5] = 2;
        }
        iPosNaliv = 70;
        break;
      case 70:
        bPovorot(igPosDeg[0]);
        iPosNaliv = 100;
        break;
    }
  }
}
void vTuneFotosensR() {
  iUstDif[0] = (iUst[0] -  analogRead(1)) / 4;
  iUstDif[1] =  (iUst[1] -  analogRead(2)) / 4;
  iUstDif[2] =  (iUst[2] -  analogRead(3)) / 4;
  iUstDif[3] =  (iUst[3] -  analogRead(4)) / 4;
  iUstDif[4] =  (iUst[4] -  analogRead(5)) / 4;
  iUstDif[5] =  (iUst[5] -  analogRead(6)) / 4;
#if(_DEBAG_)
  Serial.println( iUstDif[0]);
  Serial.println( iUstDif[1]);
  Serial.println( iUstDif[2]);
  Serial.println( iUstDif[3]);
  Serial.println( iUstDif[4]);
  Serial.println( iUstDif[5]);
  Serial.println("Уставки диф");
#endif
}
