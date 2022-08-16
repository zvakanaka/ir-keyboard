#include <Arduino.h>
#include <HID-Project.h>
#include <string.h>
#include <SD.h>
#include <SPI.h> 

/*
 * IR
 * receive = pin 2
 * send    = pin 3
 */

/*
 * SD board
 * VCC -> VCC
 * D15 -> SCK
 * D14 -> MISO
 * D16 -> MOSI
 * GND -> GND
 * D4  -> CS
 */

/*
 * BA45FF00 = Power
 * B946FF00 = Mode
 * B847FF00 = Mute
 * BB44FF00 = Play/Pause
 * BF40FF00 = Previous
 * BC43FF00 = Next
 * F807FF00 = EQ
 * EA15FF00 = Volume down
 * F609FF00 = Volume up
 * E916FF00 = 0
 * E619FF00 = RPT
 * F20DFF00 = U/SD
 * F30CFF00 = 1
 * E718FF00 = 2
 * A15EFF00 = 3
 * F708FF00 = 4
 * E31CFF00 = 5
 * A55AFF00 = 6
 * BD42FF00 = 7
 * AD52FF00 = 8
 * B54AFF00 = 9
 */

#define DEBUG true
int RXLED = 17;  // The RX LED has a defined Arduino pin
#define KEY_DELAY 150
boolean factoryTestMode = false;  // NOTE: if SD fails, this is set to true
File fileHandler;

//void setup() {
 //pinMode(RXLED, OUTPUT);  // Set RX LED as an output
//}

//void loop() {
 //digitalWrite(RXLED, LOW);   // set the LED on
// delay(1000);              // wait for a second
 //digitalWrite(RXLED, HIGH);    // set the LED off
// delay(1000);              // wait for a second
//}

// select only NEC and the universal decoder for pulse width or pulse distance
// protocols
#define DECODE_NEC       // Includes Apple and Onkyo
#define DECODE_DISTANCE  // in case NEC is not received correctly

/*
 * Define macros for input and output pin etc.
 */
#include "PinDefinitionsAndMore.h"

//#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program space.
//#define EXCLUDE_EXOTIC_PROTOCOLS
//#define SEND_PWM_BY_TIMER
//#define USE_NO_SEND_PWM
//#define NO_LED_FEEDBACK_CODE // saves 500 bytes program space
//#define DEBUG // Activate this for lots of lovely debug output from the
//decoders.
#define INFO  // To see valuable informations from universal decoder for pulse
              // width or pulse distance protocols

#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000

void executeScript(String scriptFileName);
void Line(String l);
void Press(String b);

void setup() {
#if defined(_IR_MEASURE_TIMING) && defined(_IR_TIMING_TEST_PIN)
  pinMode(_IR_TIMING_TEST_PIN, OUTPUT);
#endif

  Serial.begin(9600);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || \
  defined(SERIAL_USB) || defined(SERIALUSB_PID) ||                  \
  defined(ARDUINO_attiny3217)
  delay(4000);  // To be able to connect Serial monitor after reset or power up
                // and before first print out. Do not wait for an attached
                // Serial Monitor!
#endif
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__
        "\r\nUsing library version " VERSION_IRREMOTE));

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin
  // from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

#if defined(IR_SEND_PIN)
  IrSender.begin();  // Start with IR_SEND_PIN as send pin and enable feedback
                     // LED at default feedback LED pin
#else
  IrSender.begin(IR_SEND_PIN,
      ENABLE_LED_FEEDBACK);  // Specify send pin and enable feedback
                             // LED at default feedback LED pin
#endif

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.print(F("at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
  Serial.println(IR_RECEIVE_PIN_STRING);
#else
  Serial.println(IR_RECEIVE_PIN);
#endif
  Serial.print(F("Ready to send IR signals at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
  Serial.println(IR_SEND_PIN_STRING);
#else
  Serial.println(IR_SEND_PIN);
#endif

#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604
                        // For esp32 we use PWM generation by ledcWrite() for each pin.
#if !defined(SEND_PWM_BY_TIMER) && !defined(USE_NO_SEND_PWM) && !defined(ESP32)
  /*
   * Print internal software PWM generation info
   */
  IrSender.enableIROut(
      38);  // Call it with 38 kHz to initialize the values printed below
  Serial.print(F("Send signal mark duration is "));
  Serial.print(IrSender.periodOnTimeMicros);
  Serial.print(F(" us, pulse correction is "));
  Serial.print(IrSender.getPulseCorrectionNanos());
  Serial.print(F(" ns, total period is "));
  Serial.print(IrSender.periodTimeMicros);
  Serial.println(F(" us"));
#endif

  // infos for receive
  Serial.print(RECORD_GAP_MICROS);
  Serial.println(
      F(" us is the (minimum) gap, after which the start of a new IR packet is "
        "assumed"));
  Serial.print(MARK_EXCESS_MICROS);
  Serial.println(
      F(" us are subtracted from all marks and added to all spaces for "
        "decoding"));
#endif

  if (!SD.begin(4)) {
    Serial.println("SD FAILURE");
    Serial.println("Falling back to factory test mode (1,2,3,4,5,6,7,8)");
    factoryTestMode = true;
  }
}

uint16_t sAddress = 0x0102;
uint8_t sCommand = 0x34;
uint8_t sRepeats = 1;

/*
 * Send NEC IR protocol
 */
void send_ir_data() {
  Serial.print(F("Sending: 0x"));
  Serial.print(sAddress, HEX);
  Serial.print(sCommand, HEX);
  Serial.println(sRepeats, HEX);

  // clip repeats at 4
  if (sRepeats > 4) {
    sRepeats = 4;
  }
  // Results for the first loop to: Protocol=NEC Address=0x102 Command=0x34
  // Raw-Data=0xCB340102 (32 bits)
  IrSender.sendNEC(sAddress, sCommand, sRepeats);
}



void receive_ir_data() {
  if (IrReceiver.decode()) {
    Serial.print(F("Decoded protocol: "));
    Serial.print(getProtocolString(IrReceiver.decodedIRData.protocol));
    Serial.print(F(", decoded raw data: "));
    Serial.print(IrReceiver.decodedIRData.decodedRawData, HEX);
    Serial.print(F(", decoded address: "));
    Serial.print(IrReceiver.decodedIRData.address, HEX);
    Serial.print(F(", decoded command: "));
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    String fileNameHexPart = String(IrReceiver.decodedIRData.decodedRawData, HEX);
    fileNameHexPart.toUpperCase();
    String fileName = fileNameHexPart + ".txt";
    executeScript(fileName);

    IrReceiver.resume();
  }
}

void loop() {
  delay((RECORD_GAP_MICROS / 1000) + 5);
  receive_ir_data();

  delay(100);  // Loop delay
}

void executeScript(String scriptFileName) {
  char buf[30];
  scriptFileName.toCharArray(buf, scriptFileName.length() + 1);
  #ifdef DEBUG
    Serial.print("LOOKING FOR FILE ");
    Serial.println(buf);
  #endif
  fileHandler = SD.open(buf);
  if (fileHandler) {
    #ifdef DEBUG
      Serial.println("FOUND FILE " + scriptFileName);
    #endif
    Keyboard.begin();
    Consumer.begin(); // Initializes the media keyboard

    String line = "";

    while (fileHandler.available()) {
      char m = fileHandler.read();
      if (m == '\n') {
        #ifdef DEBUG
          Serial.println("   " + line);
        #endif
        Line(line);
        line = "";
      } else if ((int)m != 13) {
        line += m;
      }
    }
    Line(line);

    fileHandler.close();
  } else {
    Serial.println("FILE NOT FOUND " + scriptFileName);
  }

  Keyboard.end();
}

void Line(String l) {
  int space_1 = l.indexOf(" ");
  if (space_1 == -1) {
    Press(l);
  } else if (l.substring(0, space_1) == "STRING") {
    Keyboard.print(l.substring(space_1 + 1));
  } else if (l.substring(0, space_1) == "DELAY") {
    int delaytime = l.substring(space_1 + 1).toInt();
    delay(delaytime);
  } else if (l.substring(0, space_1) == "REM") {
  } else {
    String remain = l;

    while (remain.length() > 0) {
      int latest_space = remain.indexOf(" ");
      if (latest_space == -1) {
        Press(remain);
        remain = "";
      } else {
        Press(remain.substring(0, latest_space));
        remain = remain.substring(latest_space + 1);
      }
      delay(5);
    }
  }

  Keyboard.releaseAll();
}

// add support for keys from here as needed https://github.com/NicoHood/HID/blob/master/src/HID-APIs/ConsumerAPI.h
void Press(String b) {
  if (b.length() == 1) {
    char c = b[0];
    Keyboard.press(c);
  } else if (b.equals("MEDIA_PAUSE")) {
    Consumer.write(MEDIA_PAUSE);
  } else if (b.equals("MEDIA_PREVIOUS")) {
    Consumer.write(MEDIA_PREVIOUS);
  } else if (b.equals("MEDIA_NEXT")) {
    Consumer.write(MEDIA_NEXT);
  } else if (b.equals("MEDIA_VOLUME_MUTE")) {
    Consumer.write(MEDIA_VOLUME_MUTE);
  } else if (b.equals("MEDIA_VOL_MUTE")) {
    Consumer.write(MEDIA_VOL_MUTE);
  } else if (b.equals("MEDIA_VOLUME_UP")) {
    Consumer.write(MEDIA_VOLUME_UP);
  } else if (b.equals("MEDIA_VOL_UP")) {
    Consumer.write(MEDIA_VOL_UP);
  } else if (b.equals("MEDIA_VOLUME_DOWN")) {
    Consumer.write(MEDIA_VOLUME_DOWN);
  } else if (b.equals("MEDIA_VOL_DOWN")) {
    Consumer.write(MEDIA_VOL_DOWN);
  } else if (b.equals("CONSUMER_BRIGHTNESS_UP")) {
    Consumer.write(CONSUMER_BRIGHTNESS_UP);
  } else if (b.equals("CONSUMER_BRIGHTNESS_DOWN")) {
    Consumer.write(CONSUMER_BRIGHTNESS_DOWN);
  } else if (b.equals("CONSUMER_SCREENSAVER")) {
    Consumer.write(CONSUMER_SCREENSAVER);
  } else if (b.equals("MEDIA_PLAY_PAUSE")) {
    Consumer.write(MEDIA_PLAY_PAUSE);
  }  else if (b.equals("ENTER")) {
    Keyboard.press(KEY_RETURN);
  } else if (b.equals("CTRL")) {
    Keyboard.press(KEY_LEFT_CTRL);
  } else if (b.equals("SHIFT")) {
    Keyboard.press(KEY_LEFT_SHIFT);
  } else if (b.equals("ALT")) {
    Keyboard.press(KEY_LEFT_ALT);
  } else if (b.equals("GUI")) {
    Keyboard.press(KEY_LEFT_GUI);
  } else if (b.equals("UP") || b.equals("UPARROW")) {
    Keyboard.press(KEY_UP_ARROW);
  } else if (b.equals("DOWN") || b.equals("DOWNARROW")) {
    Keyboard.press(KEY_DOWN_ARROW);
  } else if (b.equals("LEFT") || b.equals("LEFTARROW")) {
    Keyboard.press(KEY_LEFT_ARROW);
  } else if (b.equals("RIGHT") || b.equals("RIGHTARROW")) {
    Keyboard.press(KEY_RIGHT_ARROW);
  } else if (b.equals("DELETE")) {
    Keyboard.press(KEY_DELETE);
  } else if (b.equals("PAGEUP")) {
    Keyboard.press(KEY_PAGE_UP);
  } else if (b.equals("PAGEDOWN")) {
    Keyboard.press(KEY_PAGE_DOWN);
  } else if (b.equals("HOME")) {
    Keyboard.press(KEY_HOME);
  } else if (b.equals("ESC")) {
    Keyboard.press(KEY_ESC);
  } else if (b.equals("INSERT")) {
    Keyboard.press(KEY_INSERT);
  } else if (b.equals("TAB")) {
    Keyboard.press(KEY_TAB);
  } else if (b.equals("END")) {
    Keyboard.press(KEY_END);
  } else if (b.equals("CAPSLOCK")) {
    Keyboard.press(KEY_CAPS_LOCK);
  } else if (b.equals("F1")) {
    Keyboard.press(KEY_F1);
  } else if (b.equals("F2")) {
    Keyboard.press(KEY_F2);
  } else if (b.equals("F3")) {
    Keyboard.press(KEY_F3);
  } else if (b.equals("F4")) {
    Keyboard.press(KEY_F4);
  } else if (b.equals("F5")) {
    Keyboard.press(KEY_F5);
  } else if (b.equals("F6")) {
    Keyboard.press(KEY_F6);
  } else if (b.equals("F7")) {
    Keyboard.press(KEY_F7);
  } else if (b.equals("F8")) {
    Keyboard.press(KEY_F8);
  } else if (b.equals("F9")) {
    Keyboard.press(KEY_F9);
  } else if (b.equals("F10")) {
    Keyboard.press(KEY_F10);
  } else if (b.equals("F11")) {
    Keyboard.press(KEY_F11);
  } else if (b.equals("F12")) {
    Keyboard.press(KEY_F12);
  } else if (b.equals("SPACE")) {
    Keyboard.press(' ');
  } else if (b.startsWith("0x")) {  
    // convert hex strings to integers (integers are the same as raw hex to HID-Project)
    // possibilities:
    //   https://github.com/NicoHood/HID/blob/master/src/KeyboardLayouts/ImprovedKeylayouts.h
    //   https://github.com/NicoHood/HID/blob/master/src/HID-APIs/ConsumerAPI.h
    //   http://www.freebsddiary.org/APC/usb_hid_usages.php
    // https://github.com/NicoHood/HID/blob/4bf6cd6167b1152a292bbc62793a85ab69070895/examples/Keyboard/ImprovedKeyboard/ImprovedKeyboard.ino#L43
    Keyboard.write(KeyboardKeycode((int)strtol(b.c_str(), 0, 16)));
  }
}


