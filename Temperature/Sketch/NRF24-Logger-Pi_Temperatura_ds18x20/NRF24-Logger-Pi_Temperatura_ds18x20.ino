/*
  Sensor digispark remoto de temperatura usando el sensor DS18B20, DS18S20 o DS1822 y NRF24L01+ para nrf24-Logger-Pi (o nrf24-Logger-OrangeZero)

  - Configuración de ID de Radio/Nodo para identificación el logs remotos
  - Configuración del tiempo entre transmisiones en segundos
  - Información transmitida en grados celsius (float) 
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
#include <OneWire.h>

// Module connection 2-Pin Hookup Guide on https://github.com/dparson55/NRFLite
#define PIN_RADIO_MOMI 0
#define PIN_RADIO_SCK 2

// Intentos de re-envio
#define intentos 5

// Pin Onewire
OneWire  ds(5);  // a 4.7K resistor is necessary (R4 in PCB)
// pin 5 disgispark, pin reset por defecto, cuidado, flashear antes con un programador ISP http://thetoivonen.blogspot.com/2015/12/fixing-pin-p5-or-6-on-digispark-clones.html
// para ver configuración -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -c avrisp -p attiny85 -n
// para cambiar el fusible -> .\avrdude.exe -P COM5 -C C:\arduino-1.8.5\hardware\tools\avr\etc\avrdude.conf -b 19200 -p attiny85 -c avrisp  -U hfuse:w:0x5F:m

//Definimos pin led estado comunicación
#define led 1

float tempC = 0.0; //Valor en celsius.
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];

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

  if (currentMillis < previousMillisPeriodical) { // en caso de estar mucho tiempo encendido, conpensar puesta a cero millis
    previousMillisPeriodical = 0;
  }

  /*************************** Comprobamos si hay que mandar resultado **********************/
  if (currentMillis - previousMillisPeriodical > (PERIODICAL * 1000)) {
    previousMillisPeriodical = currentMillis;

    if ( !ds.search(addr)) {
      //No more addresses.
      ds.reset_search();
      delay(250);
      return;
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      //CRC is not valid!
      return;
    }

    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        //Chip = DS18S20 or old DS1820
        type_s = 1;
        break;
      case 0x28:
        //Chip = DS18B20
        type_s = 0;
        break;
      case 0x22:
        //Chip = DS1822
        type_s = 0;
        break;
      default:
        //Device is not a DS18x20 family device
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    for (byte i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    tempC = (float)raw / 16.0;
    //float fahrenheit = tempC * 1.8 + 32.0;

    _radioData.FromRadioId = RADIO_ID;
    _radioData.DataType = 'T';  // Tipo de dato Temperatura
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
    delay(10);
    // apagamos led
    digitalWrite(led, LOW);
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
