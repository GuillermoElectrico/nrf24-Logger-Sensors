/*
  Sensor digispark remoto de temperatura usando el sensor LM35 y NRF24L01+ para nrf24-Logger-Pi (o nrf24-Logger-OrangeZero)
  
  - Configuración de ID de Radio/Nodo para identificación el logs remotos
  - Configuración del tiempo entre transmisiones en segundos
  - Información transmitida en grados celsius(float)
*/
//------------------------------ SOLO configurar ESTO --------------------------------

#define RADIO_ID 3                // Radio/Nodo ID para identificación en log remoto
#define DESTINATION_RADIO_ID 1    // Id of the radio we will transmit to.
#define PERIODICAL 30             // Tiempo en segundos entre envíos via RF
#define BAUDRATE BITRATE2MBPS     // BITRATE2MBPS, BITRATE1MBPS, BITRATE250KBPS
#define CHANNEL 100               // 0-125 (2400 - 2525 MHz)

//---------------------------------------------------------------------------------

#include <NRFLite.h>
#include <avr/wdt.h>

// Module connection 2-Pin Hookup Guide on https://github.com/dparson55/NRFLite
#define PIN_RADIO_MOMI 0
#define PIN_RADIO_SCK 2

// Intentos de re-envio
#define intentos 5

// Entrada analógica
#define AInput A0 // pin 5 disgispark, reset cuidado de flashear antes con un programador ISP http://thetoivonen.blogspot.com/2015/12/fixing-pin-p5-or-6-on-digispark-clones.html
// para ver configuración -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -c avrisp -p attiny85 -n
// para cambiar el fusible -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -p attiny85 -c avrisp  -U hfuse:w:0x5F:m

//Definimos pin led estado comunicación
#define led 1

float tempC = 0.0; //Valor en ºc.
int AN0[50];//entrada analógica, 50 lecturas por segundo, conversion, precision de 0 a 1023.
float ANT; // total y media de las 50 lecturas/segundo de la entrada analógica
byte analogCounter = 0;

unsigned long previousMillis = 0; // buffer de guardado para contar tiempo transcurrido, evitar delays innecesarios
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

  analogReference(INTERNAL);

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

  // apagamos led
  digitalWrite(led, LOW);
  delay(1000);
  wdt_enable(WDTO_8S);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis < previousMillis || currentMillis < previousMillisPeriodical) { // en caso de estar mucho tiempo encendido, conpensar puesta a cero millis
    previousMillis = 0;
    previousMillisPeriodical = 0;
  }

  if (currentMillis - previousMillis > 20) {  // 50 lecturas cada segundo 1000/50 = 20ms
    previousMillis = currentMillis;
    AN0[analogCounter] = analogRead(AInput); //sera un valor comprendido entre 0 y 1023.
    analogCounter++;
    //
    if (analogCounter >= 50) {
      ANT = 0.0;
      for (byte a = 0; a < analogCounter; a++) {
        ANT = ANT + AN0[a]; // sumo el total de las lecturas
      }
      ANT = ANT / analogCounter; // hago la media de las muestras sumadas
      analogCounter = 0; // pongo a cero para la siguiente ronda de lecturas
      //
      //Obtenemos temperatura equivalente a valor leido en entrada analógica
      tempC = (1.1 * ANT * 100.0) / 1024.0;

      /*************************** Comprobamos si hay que mandar resultado **********************/
      if (currentMillis - previousMillisPeriodical > (PERIODICAL * 1000)) {
        previousMillisPeriodical = currentMillis;

        _radioData.FromRadioId = RADIO_ID;
        _radioData.DataType = 'A';  // Tipo de dato Analógico
        _radioData.InputNumber = 0; // cero ya que no puede ser ampliado
        _radioData.RadioDataLong = 0;
        _radioData.RadioDataFloat = tempC;
        // _radioData.RadioDataFloat = 1.23456789;

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
      }
      delay(10);
      // apagamos led
      digitalWrite(led, LOW);
    }
  }
  // comprobamos si hay algúna comunicacion pendiente vía radio
  while (_radio.hasData())
  {
    _radio.readData(&_radioData); // Note how '&' must be placed in front of the variable name.

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
    delay(10);
    // apagamos led
    digitalWrite(led, LOW);
  }
  wdt_reset();
}
