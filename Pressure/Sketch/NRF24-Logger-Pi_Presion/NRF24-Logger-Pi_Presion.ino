/*
  Sensor digispark remoto de presión usando el sensor 5V 1.2Mpa y NRF24L01+ para nrf24-Logger-Pi (o nrf24-Logger-OrangeZero)

  - Medida de 50 veces por segundo (para mitigar interferencias y posible errores de lectura)
  - Configuración de ID de Radio/Nodo para identificación el logs remotos
  - Configuración del tiempo entre transmisiones en segundos
  - Información transmitida en KPa (Bar = KPa / 100) (float)
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

/*Maxima y mínima entrada analogica conversion en milivoltios*/
#define AnConverMax 4475.0 //Entrada analogica conversion maxima, 1.2MPa con 4.5V. 
#define AnCaliMin 475.0 //Calibración entada analógica sensor empieza en 0Pa con 0,5V.

/*Máxima y mínima medición del sensor en KPa*/
#define KPaSensorMax 1200.0 //Medida máxima de KPa del sensor.
#define KPaSensorMin 0.0 //Medida mínima de KPa del sensor.

float KPa = 0.0;//Valor en KPa.
int AN0[50];//entrada analógica, 50 lecturas por segundo, conversion, precision de 0 a 1023.
float ANT; // total y media de las 50 lecturas/segundo de la entrada analógica
float Tension = 0.0; //voltage en milivoltios equivalente entrada analógica
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

struct RadioPacketRepeater // Any packet up to 32 bytes can be sent.
{
  byte FromRadioIdR;
  char DataTypeR;
  byte InputNumberR;
  long RadioDataLongR;
  float RadioDataFloatR;
  unsigned long FailedTxCountR;
};

NRFLite _radio;
RadioPacket _radioData;
RadioPacketRepeater _radioDataRepeater;

void setup() {
  wdt_disable();
  // encendemos led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

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
      //Obtenemos tensión equivalente entrada analógica (0-5000mV)
      Tension = map(ANT, 0.0, 1023.0, 0.0, 5000.0); //mapeamos valor entrada analógica 0-1023 a 0-5000mV)
      if (Tension < AnCaliMin)
        Tension = AnCaliMin;
      if (Tension > AnConverMax)
        Tension = AnConverMax;
      //Obtenemos presión equivalente a tensión leida en entrada analógica
      KPa = map(Tension, AnCaliMin, AnConverMax, KPaSensorMin, KPaSensorMax); //mapeamos valor tensión entrada analógica (500-4500mV) a 0-1200 KPa.
      /*************************** Comprobamos si hay que mandar resultado **********************/
      if (currentMillis - previousMillisPeriodical > (PERIODICAL * 1000)) {
        previousMillisPeriodical = currentMillis;

        _radioData.DataType = 'A';  // Tipo de dato Analógico
        _radioData.InputNumber = 0; // cero ya que no puede ser ampliado
        _radioData.RadioDataLong = 0;
        _radioData.RadioDataFloat = KPa;
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
    _radio.readData(&_radioDataRepeater); // Note how '&' must be placed in front of the variable name.

    byte retries = intentos;

    while (retries > 0) {
      wdt_reset();
      retries--;
      if (_radio.send(DESTINATION_RADIO_ID, &_radioDataRepeater, sizeof(_radioDataRepeater), NRFLite::REQUIRE_ACK)) // Note how '&' must be placed in front of the variable name.
      {
        // Encendemos led un momento para indicar envio correcto
        digitalWrite(led, HIGH);
        retries = 0;
      }
      else
      {
        _radioDataRepeater.FailedTxCountR++;
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
