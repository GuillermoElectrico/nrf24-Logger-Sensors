/*
  Monitor digispark remoto de entradas digitales y NRF24L01+ para nrf24-Logger-Pi (o nrf24-Logger-OrangeZero)

  - Envia datos ante cambios en las entradas y/o de manera periódica
  - Configuración de ID de Radio/Nodo para identificación el logs remotos
  - Información transmitida en formato "binario humano" (long) (representación decimal de un valor binario 00000000)
*/
//------------------------------ SOLO configurar ESTO --------------------------------

#define RADIO_ID 1              // Radio/Nodo ID para identificación en log remoto
#define DESTINATION_RADIO_ID 0  // Id of the radio we will transmit to.
#define PERIODICAL 1            // Tiempo en segundos entre envíos via RF (0=deshabilitado)
#define BAUDRATE BITRATE2MBPS   // BITRATE2MBPS, BITRATE1MBPS, BITRATE250KBPS
#define CHANNEL 100             // 0-125 (2400 - 2525 MHz)

//---------------------------------------------------------------------------------

#include <NRFLite.h>
#include <avr/wdt.h>

// Module connection 2-Pin Hookup Guide on https://github.com/dparson55/NRFLite
#define PIN_RADIO_MOMI 0
#define PIN_RADIO_SCK 2

// Intentos de re-envio
#define intentos 3

//Definimos pin led estado comunicación
#define led 1

//define where your pins are control to read CD4014
#define latchPin 4
#define dataPin 6
#define clockPin 5 // pin 5 disgispark, reset cuidado de flashear antes con un programador ISP http://thetoivonen.blogspot.com/2015/12/fixing-pin-p5-or-6-on-digispark-clones.html
// para ver configuración -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -c avrisp -p attiny85 -n
// para cambiar el fusible -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -p attiny85 -c avrisp  -U hfuse:w:0x5F:m

#define delayShift 50  // tiempo entre pulsos y reloj en microsegundos (velocidad bus lectura)
//definimos registros y los ponemos a 255 (entradas pullup)
byte switchVar = 255;
byte state = 255;

boolean sendNRF = false;
unsigned long previousMillisPeriodical = 0;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
  byte FromRadioId;
  char DataType;
  byte InputNumber;
  long RadioDataLong;
  float RadioDataFloat;
  unsigned long FailedTxCount;
};

NRFLite _radio;
RadioPacket _radioData;

void setup() {
  wdt_disable();
  // encendemos led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  // definimos ping sio
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);

  if (!_radio.initTwoPin(RADIO_ID, PIN_RADIO_MOMI, PIN_RADIO_SCK, NRFLite::BAUDRATE, CHANNEL))
  {
    while (1) { // Wait here forever.
      // encendemos led rapidamentepara mostrar error
      delay(50);
      digitalWrite(led, LOW);
      delay(50);
      digitalWrite(led, HIGH);
    }
  }

  _radioData.FromRadioId = RADIO_ID;

  // apagamos led
  digitalWrite(led, LOW);
  delay(1000);
  wdt_enable(WDTO_8S);
}

void loop() {

  // comprobar si está configurado algún tiempo de actualización periódica.
  if (PERIODICAL > 0) {
    unsigned long currentMillis = millis();

    // en caso de estar mucho tiempo encendido, conpensar puesta a cero millis
    if (currentMillis < previousMillisPeriodical) {
      previousMillisPeriodical = 0;
    }

    // Mirar si es necesario enviar estado
    if (currentMillis - previousMillisPeriodical > (PERIODICAL * 1000)) {
      previousMillisPeriodical = currentMillis;
      sendNRF = true;
    }
  }

  // leemos las entradas del módulo cd4014
  // primero registramos el estado de las entradas

  digitalWrite(latchPin, HIGH);
  delayMicroseconds(delayShift);
  digitalWrite(clockPin, HIGH);
  delayMicroseconds(delayShift);
  digitalWrite(clockPin, LOW);
  delayMicroseconds(delayShift);
  digitalWrite(latchPin, LOW);
  delayMicroseconds(delayShift);

  // ahora leemos las entradas

  switchVar = shiftIn();

  // borramos datos previos
  _radioData.RadioDataLong = 0;
  // guardamos por si acaso hay que enviar el tipo como Input
  _radioData.DataType = 'M';  // Tipo de dato Módulo
  _radioData.InputNumber = 0; // cero ya que no puede ser ampliado

  // guardamos en el data en binario humano
  for (byte x = 0; x < 8; x++)
  {
    long multiplicador = 10;
    for (byte y = 0; y < x; y++)
    {
      multiplicador *= 10;
    }
    multiplicador = multiplicador / 10;

    if (!bitRead(switchVar, x))
      _radioData.RadioDataLong += multiplicador;
  }

  // comprobamos si ha cambiado desde la última vez
  if (switchVar != state) {
    // almacenamos estado
    state = switchVar;
    // mandamos enviar registro
    sendNRF = true;
  }

  if (sendNRF) {

    byte retries = intentos;

    while (retries > 0) {
      wdt_reset();
      retries--;
      if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData), NRFLite::REQUIRE_ACK)) // Note how '&' must be placed in front of the variable name.
      {
        // Encendemos led un momento para indicar envio correcto
        digitalWrite(led, HIGH);
        retries = 0;
      }
      else
      {
        _radioData.FailedTxCount++;
        // Encendemos led intermitente rápido para indicar envio infructuoso
        for (byte a = 0; a < 10; a++) {
          digitalWrite(led, !digitalRead(led));
          delay(50);
        }
      }
    }
    //marcamos enviado
    sendNRF = false;
    delay(10);
    // apagamos led
    digitalWrite(led, LOW);
  }

  wdt_reset();

}

////// ----------------------------------------shiftIn function
///// just needs the location of the data pin and the clock pin
///// it returns a byte with each bit in the byte corresponding
///// to a pin on the shift register. leftBit 7 = Pin 7 / Bit 0= Pin 0
byte shiftIn() {
  int i;
  byte temp = 0;
  byte myDataIn = 0;
  //pinMode(myClockPin, OUTPUT);  dataPin, clockPin
  //pinMode(myDataPin, INPUT);
  //we will be holding the clock pin high 8 times (0,..,7) at the
  //end of each time through the for loop
  //at the begining of each loop when we set the clock low, it will
  //be doing the necessary low to high drop to cause the shift
  //register's DataPin to change state based on the value
  //of the next bit in its serial information flow.
  //The register transmits the information about the pins from pin 7 to pin 0
  //so that is why our function counts down
  for (i = 7; i >= 0; i--)
  {
    digitalWrite(clockPin, LOW);
    delayMicroseconds(delayShift);
    temp = digitalRead(dataPin);
    if (temp) {
      myDataIn = myDataIn | (1 << i);
    }
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(delayShift);
  }
  digitalWrite(clockPin, LOW);
  return myDataIn;
}
