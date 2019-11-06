/* svalinn-firmware-esp8266 source code beginning. */
/**
 * Copyright (c) 2019 Abderraouf Adjal <abderraouf.adjal@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * Project name: svalinn-firmware-esp8266
 * Project version: v0.9.1 / 2019-09-17
 * License: The ISC license.
 * Authors information: Abderraouf Adjal <abderraouf.adjal@gmail.com>.
 *
 * Svalinn: An IoT indoor alarm system.
 * This system can be controlled by any MQTT connected device 
 * such as laptops, smartphones,  or an esp8266 device as a client 
 * for control and alarm notification.
 * 
 * Requirements
 * # Hardware:
 *   - ESP8266 (NodeMCU, D1 mini, D1 mini Pro),
 *   - PIR motion sensor AM312 or HC-SR501,
 *   - Normally closed (NC) door opening sensor(s) (a switch) to GND, 
 *   - Internet-connected Wi-Fi.
 * # Software:
 *   - ESP8266 Arduino Core, tested with v2.5.2 (https://github.com/esp8266/Arduino),
 *   - PubSubClient library, tested with v2.7.0 (https://github.com/knolleary/pubsubclient),
 *   - WiFiManager library, tested with v0.14.0 (https://github.com/tzapu/WiFiManager),
 *   - Topics created in the MQTT server.
 * 
 * # NOTES:
 *   - Recemmanded IO: Built-in LED GPIO-2/D4.
 *                     Sensors: GPIO-14/D5 & GPIO-12/D6/PWM0.
 *                     SV_ALARM_PIN GPIO-13/D7/PWM2.
 *                     Other IO you can use: GPIO-4/D2/PWM3/SDA & GPIO-5/D1/SCL.
 *   - If no PIR will be connected, wire the pin to GND.
 *   - It's possible to use more than one door opening sensor.
 *   - Usage via USB will result boot reason REASON_EXT_SYS_RST.
 *   - MQTT topics length limit in bytes is defined by (SV_MQTT_TOPIC_MAX_LEN - 1).
 *   - A free MQTT server service: Adafruit IO (https://io.adafruit.com).
 *   - A command to get X.509 SHA-1 fingerprint by OpenSSL and Bash:
 *     $ openssl s_client -connect "io.adafruit.com:8883" < /dev/null 2>/dev/null
 *         | openssl x509 -fingerprint -sha1 -noout -in /dev/stdin
 */

/** MQTT Topics structure:
 * 
 * "$MQTT_TOPICS_PREFIX+$SV_MQTT_TOPIC_CTRL"
 * MQTT device control topic, Will SUB only.
 * --------------------------------
 * # Control Security status:
 * '0': Security is OFF.
 * '1': Read SV_DOOR_SW_PIN only.
 * '2': Read SV_PIR_PIN only.
 * '3': Read SV_DOOR_SW_PIN and SV_PIR_PIN.
 * --------------------------------
 * # Control alarm signal output:
 * 'L': Turn OFF alarm.
 * 'R': Turn ON alarm.
 * --------------------------------
 * # Ping the system for sync:
 * 'P': Force publish SV_security_state and SV_alarm_state.
 * ================================================================
 * 
 * "$MQTT_TOPICS_PREFIX+$SV_MQTT_TOPIC_SECURITY"
 * MQTT security status topic, will PUP only.
 * --------------------------------
 * '0': Security is OFF.
 * '1': Read SV_DOOR_SW_PIN only.
 * '2': Read SV_PIR_PIN only.
 * '3': Read SV_DOOR_SW_PIN and SV_PIR_PIN.
 * ================================================================
 * 
 * "$MQTT_TOPICS_PREFIX+$SV_MQTT_TOPIC_ALARM"
 * MQTT alarm status topic, will PUP only.
 * --------------------------------
 * 'L': No alert from sensors.
 * 'A': Door is open.
 * 'B': PIR detect movement.
 * 'C': Door is open and PIR detect movement.
 * 'R': Alarm turned-on remotely.
 */

/** TODO:
 * - Encrypt the conf AP for the conf info security in RF.
 * - MQTT conn: Study to add "Offline Will" payload 'W' to alarm ctrl topic.
 * - Optimize offline situation to not reboot system.
 * - Study the usage of optimistic_yield().
*/

#include <EEPROM.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>

/************************* USER CONFIGURATION *********************************/
/* Semantic Versioning: MAJOR, MINOR, PATCH. */
static const uint8_t SV_version[3] = { 0, 9, 1 };
//#define SVALINN_IN_DEBUG

/* Sensors & alarm IO pins:
 * - Define SV_PIR_HCSR501 if the PIR HC-SR501 will be used.
 * - Define SV_INVERTED_LED if the built-in LED will be used,
 *   this is to invert its GPIO pin output states.
*/
//#define SV_PIR_HCSR501
#define SV_INVERTED_LED

#define SV_LED_PIN   2
#define SV_ALARM_PIN 13
#define SV_PIR_PIN   14
#define SV_DOOR_SW_PIN 12

static const size_t SV_MQTT_SERVER_MAX_LEN = 61;  /* 60 chars + '\0' */
static const size_t SV_MQTT_PORT_MAX_LEN   = 2;   /* uint16_t */
static const size_t SV_MQTT_USER_MAX_LEN   = 65;  /* 64 chars + '\0' */
static const size_t SV_MQTT_KEY_MAX_LEN    = 65;  /* 64 chars + '\0' */
static const size_t SV_MQTT_WITH_TLS_MAX_LEN = 1; /* uint8_t */
static const size_t SV_MQTT_TLS_FINGERPRINT_MAX_LEN = 60; /* 59 chars + '\0' formated SHA-1 */
static const size_t SV_MQTT_TOPICS_PREFIX_MAX_LEN = 121;  /* 120 chars + '\0' */
static const size_t SV_MQTT_TOPIC_CTRL_MAX_LEN  = 21;     /* 20 chars + '\0' */
static const size_t SV_MQTT_TOPIC_SEC_MAX_LEN   = SV_MQTT_TOPIC_CTRL_MAX_LEN;
static const size_t SV_MQTT_TOPIC_ALARM_MAX_LEN = SV_MQTT_TOPIC_CTRL_MAX_LEN;
#define SV_MQTT_TOPIC_MAX_LEN \
  (SV_MQTT_TOPICS_PREFIX_MAX_LEN + SV_MQTT_TOPIC_CTRL_MAX_LEN - 1)
#define SV_MQTT_CTRL_TOPIC_SUB_QOS 1

#define SV_RESET_CONF_COUNT 3 /* RST button count to reset connection settings. */
#define SV_STATES_IDLE_PUB_INTERVAL_MS 120000UL /* 2m */
#define SV_STATES_ALARM_PUB_INTERVAL_MS 20000UL /* 20s */
#define SV_OFFLINE_BOOT_TIMEOUT_MS 120000UL /* 2m */
#define SV_RESET_BUTTON_TIMEOUT_MS 6000UL /* 6s */
/* To reboot for avoiding millis() rollover. */
#define SV_LONG_UPTIME_TIMEOUT_MS 2592000000UL /* 30 days */

#ifndef MQTT_MAX_PACKET_SIZE
/* Maximum packet size in bytes.
 * NOTE: The default MQTT_MAX_PACKET_SIZE in <PubSubClient.h> is only 128 bytes,
 * length(clientId+username+password) and length(topic+payload) must be less than it.
 */
# define MQTT_MAX_PACKET_SIZE 192
#else
# error "PubSubClient options are pre-defined."
#endif

#ifndef MQTT_KEEPALIVE
/* MQTT KeepAlive interval in seconds. */
# define MQTT_KEEPALIVE 75
#else
# error "MQTT_KEEPALIVE is pre-defined."
#endif

#ifndef MQTT_SOCKET_TIMEOUT
/* Socket and MQTT connect() timeout interval in seconds. */
# define MQTT_SOCKET_TIMEOUT 40
#else
# error "MQTT_SOCKET_TIMEOUT is pre-defined."
#endif

//#ifndef MQTT_MAX_TRANSFER_SIZE
///* Define to limit how much data in bytes is passed to the network client. */
//# define MQTT_MAX_TRANSFER_SIZE WIFICLIENT_MAX_PACKET_SIZE /* == TCP_MSS */
//#else
//# error "MQTT_MAX_TRANSFER_SIZE is pre-defined."
//#endif

//#ifndef WIFI_MANAGER_MAX_PARAMS
//# define WIFI_MANAGER_MAX_PARAMS 11
//#else
//# error "WIFI_MANAGER_MAX_PARAMS is pre-defined."
//#endif
/******************************************************************************/

#include <PubSubClient.h>
#include <WiFiManager.h>

/* At least 60s of delay To calibrate PIR HC-SR501 */
#ifndef SV_PIR_HCSR501
# define SV_PIR_INIT_INTERVAL_MS 0
#else
# define SV_PIR_INIT_INTERVAL_MS 70000UL /* 70s */
#endif

/* NOTE: There is no EEPROM in ESP8266, Flash memory is emulated as EEPROM. */
static const int SV_DEVICE_CONF_STATE_EEPROM_ADDR = 0;
static const int SV_RESET_COUNT_EEPROM_ADDR    = 1;
static const int SV_ALARM_STATE_EEPROM_ADDR    = 2;
static const int SV_SECURITY_STATE_EEPROM_ADDR = 3;
static const int SV_MQTT_SERVER_EEPROM_ADDR = 4;
static const int SV_MQTT_PORT_EEPROM_ADDR = 
  SV_MQTT_SERVER_EEPROM_ADDR + SV_MQTT_SERVER_MAX_LEN;
static const int SV_MQTT_USER_EEPROM_ADDR = 
  SV_MQTT_PORT_EEPROM_ADDR + SV_MQTT_PORT_MAX_LEN;
static const int SV_MQTT_KEY_EEPROM_ADDR = 
  SV_MQTT_USER_EEPROM_ADDR + SV_MQTT_USER_MAX_LEN;
static const int SV_MQTT_WITH_TLS_EEPROM_ADDR = 
  SV_MQTT_KEY_EEPROM_ADDR + SV_MQTT_KEY_MAX_LEN;
static const int SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR = 
  SV_MQTT_WITH_TLS_EEPROM_ADDR + SV_MQTT_WITH_TLS_MAX_LEN;
static const int SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR = 
  SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR + SV_MQTT_TLS_FINGERPRINT_MAX_LEN;
static const int SV_MQTT_TOPIC_CTRL_EEPROM_ADDR = 
  SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR + SV_MQTT_TOPICS_PREFIX_MAX_LEN;
static const int SV_MQTT_TOPIC_SEC_EEPROM_ADDR = 
  SV_MQTT_TOPIC_CTRL_EEPROM_ADDR + SV_MQTT_TOPIC_CTRL_MAX_LEN;
static const int SV_MQTT_TOPIC_ALARM_EEPROM_ADDR = 
  SV_MQTT_TOPIC_SEC_EEPROM_ADDR + SV_MQTT_TOPIC_SEC_MAX_LEN;

#define SV_MQTT_EEPROM_BEGIN_ADDR SV_MQTT_SERVER_EEPROM_ADDR
#define SV_MQTT_EEPROM_END_ADDR \
  (SV_MQTT_TOPIC_ALARM_EEPROM_ADDR + SV_MQTT_TOPIC_ALARM_MAX_LEN - 1)
#define SV_EEPROM_SIZE 512 /* Max size is SPI_FLASH_SEC_SIZE */

#define SV_SECURITY_STATE_OFF    '0' /* Default 1st run option. */
#define SV_SECURITY_STATE_SWITCH '1'
#define SV_SECURITY_STATE_PIR    '2'
#define SV_SECURITY_STATE_SWITCH_PIR '3'
#define SV_ALARM_STATE_OFF    'L' /* Default 1st run option. */
#define SV_ALARM_STATE_SWITCH 'A'
#define SV_ALARM_STATE_PIR    'B'
#define SV_ALARM_STATE_SWITCH_PIR 'C'
#define SV_ALARM_STATE_REMOTE     'R'
#define SV_FORCE_PUBLISH_STATES   'P'

volatile unsigned long SV_states_next_publish_time = 0;
volatile uint8_t SV_security_state = SV_SECURITY_STATE_SWITCH_PIR;
volatile uint8_t SV_alarm_state    = SV_ALARM_STATE_OFF;
volatile bool SV_states_scheduled_publish = true;
volatile bool SV_MQTT_functions_no_error  = true;
volatile bool SV_device_is_online   = false;
volatile bool SV_PIR_is_initialized = false;
volatile bool SV_sensors_interrupt_attached = false;

uint32_t chip_ID, mem_ID;
char SV_device_ID[21];
char SV_MQTT_topic_buf[SV_MQTT_TOPIC_MAX_LEN];

Ticker SV_PIR_init_ticker;
Ticker SV_LED_indicator_ticker;
WiFiClient SV_PacketsClient;
BearSSL::WiFiClientSecure SV_PacketsClientSecure;
PubSubClient SV_PubSubClient;


uint8_t SV_EEPROM_state_read(const int addr)
{
  return EEPROM.read(addr);
}

void SV_EEPROM_state_write(const int addr,
                           const uint8_t state)
{
  EEPROM.write(addr, state);
  volatile bool r = EEPROM.commit();
  if (! r) {
    /* Error, retry writing many times. */
    for (uint8_t i = 0; (! r) && (i < UINT8_MAX); i++) {
      r = EEPROM.commit();
    }
  }
}

bool SV_valid_security_state(const uint8_t s)
{
  if ((s == SV_SECURITY_STATE_OFF)
   || (s == SV_SECURITY_STATE_SWITCH)
   || (s == SV_SECURITY_STATE_PIR)
   || (s == SV_SECURITY_STATE_SWITCH_PIR)) 
  {
    return true;
  }
  return false;
}

bool SV_valid_alarm_state(const uint8_t s)
{
  if ((s == SV_ALARM_STATE_OFF)
   || (s == SV_ALARM_STATE_SWITCH)
   || (s == SV_ALARM_STATE_PIR)
   || (s == SV_ALARM_STATE_SWITCH_PIR)
   || (s == SV_ALARM_STATE_REMOTE)) 
  {
    return true;
  }
  return false;
}

void ICACHE_RAM_ATTR LED_indicator_ctrl(void)
{
  if ((SV_alarm_state != SV_ALARM_STATE_OFF)
   || (! SV_PIR_is_initialized) 
   || (! SV_MQTT_functions_no_error)
   || (! SV_device_is_online))
  {
    /* Flip output state for blinking animation. */
    digitalWrite(SV_LED_PIN, ! digitalRead(SV_LED_PIN));
  }
  else {
    /* LED in stable state, either HIGH or LOW. */
#ifndef SV_INVERTED_LED
    digitalWrite(SV_LED_PIN,
      (SV_security_state == SV_SECURITY_STATE_OFF) ? LOW : HIGH);
#else
    /* NOTE: Usally the built-in LED state is opposite of its GPIO pin output. */
    digitalWrite(SV_LED_PIN,
      (SV_security_state == SV_SECURITY_STATE_OFF) ? HIGH : LOW);
#endif
  }
}

void ICACHE_RAM_ATTR SV_alarm_ctrl_pin(const bool state)
{
  digitalWrite(SV_ALARM_PIN, state);
}

void SV_alarm_state_ctrl(const uint8_t state)
{
  uint8_t smart_state = state;
  if ((SV_alarm_state != SV_ALARM_STATE_OFF)
   && (SV_alarm_state != SV_ALARM_STATE_REMOTE)
   && (state != SV_ALARM_STATE_OFF))
  {
    if ((state == SV_ALARM_STATE_REMOTE)) {
      /* Don't change the state.
       * To not replace active state by sensors with SV_ALARM_STATE_REMOTE */
      return ;
      /* smart_state = SV_alarm_state; */
    }
    else {
      /* Mark alarm is from both sensors. */
      smart_state = SV_ALARM_STATE_SWITCH_PIR;
    }
  }
  SV_EEPROM_state_write(SV_ALARM_STATE_EEPROM_ADDR, smart_state);
  SV_alarm_state = smart_state;
}

void SV_alarm_ctrl(const uint8_t state)
{
  SV_alarm_ctrl_pin(state != SV_ALARM_STATE_OFF);
  if (state != SV_alarm_state) {
    SV_alarm_state_ctrl(state);
    SV_states_scheduled_publish = true; /* To upload SV_alarm_state */
    /* NOTE: Here you can add ONE-TIME callback function when SV_alarm_state changed
     * such as sending GSM SMS notification or an e-mail.
     */
  }
}

/* Get current sensors states.
 * Returns:
 * SV_ALARM_STATE_OFF: No alert from sensors, 
 *   or SV_security_state is SV_SECURITY_STATE_OFF.
 * SV_ALARM_STATE_SWITCH: Door is open.
 * SV_ALARM_STATE_PIR: PIR detect movement.
 * SV_ALARM_STATE_SWITCH_PIR: Door is open and PIR detect movement.
 */
uint8_t ICACHE_RAM_ATTR get_sensors_states(void)
{
  volatile uint8_t r = SV_ALARM_STATE_OFF;
  
  if (SV_security_state == SV_SECURITY_STATE_SWITCH) {
    /* Read SV_DOOR_SW_PIN only */
    if (digitalRead(SV_DOOR_SW_PIN) == LOW) {
      r = SV_ALARM_STATE_SWITCH;
    }
  }
  else if (SV_security_state == SV_SECURITY_STATE_PIR) {
    /* Read SV_PIR_PIN only */
    if (SV_PIR_is_initialized && (digitalRead(SV_PIR_PIN) == HIGH)) {
      r = SV_ALARM_STATE_PIR;
    }
  }
  else if (SV_security_state == SV_SECURITY_STATE_SWITCH_PIR) {
    /* Read SV_DOOR_SW_PIN and SV_PIR_PIN */
    volatile bool s = (digitalRead(SV_DOOR_SW_PIN) == LOW);
    volatile bool p = (digitalRead(SV_PIR_PIN) == HIGH) && SV_PIR_is_initialized;
    if (s && p) {
      r = SV_ALARM_STATE_SWITCH_PIR;
    } else if (s) {
      r = SV_ALARM_STATE_SWITCH;
    } else if (p) {
      r = SV_ALARM_STATE_PIR;
    }
  }
  
  return r;
}

/* Get current sensors states and act based on it. */
void ICACHE_RAM_ATTR SV_capture_and_act(void)
{
  volatile uint8_t sensors_state_r = get_sensors_states();
  if (sensors_state_r != SV_ALARM_STATE_OFF) {
    SV_alarm_ctrl(sensors_state_r);
  }
}

/* Pins interrupt alarm ISR. */
void ICACHE_RAM_ATTR ISR_capture_sensors(void)
{
  ETS_GPIO_INTR_DISABLE();
  SV_capture_and_act();
  ETS_GPIO_INTR_ENABLE();
}

void SV_security_ctrl(const uint8_t state)
{
  SV_EEPROM_state_write(SV_SECURITY_STATE_EEPROM_ADDR, state);
  SV_security_state = state;
}

/* Configure relative GPIO pins. */
void SV_init_pins(void)
{
  pinMode(SV_ALARM_PIN, OUTPUT);
  SV_alarm_ctrl_pin(LOW);
  pinMode(SV_LED_PIN, OUTPUT);
  pinMode(SV_PIR_PIN, INPUT);
  pinMode(SV_DOOR_SW_PIN, INPUT_PULLUP);
}

/* Calibrate sensors and attach interruption */
void SV_init_sensors(void)
{
  /* If the time is out, or no power outage in PIR (not normal power-on boot). */
  /* NOTE: USB usage with computer might boot with RST button reason. */
  if ((millis() >= SV_PIR_INIT_INTERVAL_MS)
        || (system_get_rst_info()->reason != REASON_DEFAULT_RST))
  {
    SV_PIR_is_initialized = true;
    SV_PIR_init_ticker.detach();
    /*if (SV_PIR_init_ticker.active()) {
      SV_PIR_init_ticker.detach();
    }*/
  }

  /* Force read sensors state,
   * no "CHANGE interruption" is possible case if a door(s) open after power outage */
  SV_capture_and_act();

  if (! SV_sensors_interrupt_attached) {
    attachInterrupt(digitalPinToInterrupt(SV_DOOR_SW_PIN), ISR_capture_sensors, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SV_PIR_PIN), ISR_capture_sensors, CHANGE);
    SV_sensors_interrupt_attached = true;
  }
  
  if (! SV_PIR_is_initialized) {
    /* To self-call this function again after SV_PIR_INIT_INTERVAL_MS. */
    SV_PIR_init_ticker.once_ms(SV_PIR_INIT_INTERVAL_MS, SV_init_sensors);
  }
}

void SV_WiFi_connect(void)
{
  /* SDK will auto-connect ... */
  WiFi.mode(WIFI_STA); /* Force station mode. */
  WiFi.setSleepMode(WIFI_NONE_SLEEP); /* Turn off Wi-Fi power saving. */
  delay(500);
  /* Work offline while device is not connected to WiFi yet */
  while (WiFi.status() != WL_CONNECTED) {
    /* If still not connected to the network (Wi-Fi) for long time */
    if (millis() >= SV_OFFLINE_BOOT_TIMEOUT_MS) {
      SV_offline_handle();
    }
    delay(50);
  }
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("# SV_WiFi_connect(): Connected in (ms): "));
  Serial.println(millis());
  Serial.println(WiFi.localIP());
#endif
}

/* Setup connection with MQTT server, and it will reboot the device if failed to. */
void SV_MQTT_connect(void)
{
  const char* MQTT_SERVER =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_SERVER_EEPROM_ADDR;
  const char* MQTT_USER =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_USER_EEPROM_ADDR;
  const char* MQTT_KEY =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_KEY_EEPROM_ADDR;
  const char* MQTT_TLS_FINGERPRINT =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR;
  const uint16_t MQTT_PORT =
    ((uint16_t)EEPROM.read(SV_MQTT_PORT_EEPROM_ADDR) << 8)
    | EEPROM.read(SV_MQTT_PORT_EEPROM_ADDR + 1);
  const uint8_t MQTT_WITH_TLS = EEPROM.read(SV_MQTT_WITH_TLS_EEPROM_ADDR);

  snprintf(SV_MQTT_topic_buf, sizeof(SV_MQTT_topic_buf),
    "%s%s",
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_CTRL_EEPROM_ADDR));

#ifdef SVALINN_IN_DEBUG
  Serial.println(F("# SV_MQTT_connect():"));
  Serial.print(F("  MQTT_SERVER: "));
  Serial.println(MQTT_SERVER);
  Serial.print(F("  MQTT_PORT: "));
  Serial.println(MQTT_PORT);
  Serial.print(F("  MQTT_USER: "));
  Serial.println(MQTT_USER);
  Serial.print(F("  MQTT_KEY: "));
  Serial.println(MQTT_KEY);
  Serial.print(F("  MQTT_WITH_TLS: "));
  Serial.println((MQTT_WITH_TLS == 'Y')? 'Y': 'N');
  Serial.print(F("  MQTT_TLS_FINGERPRINT: "));
  Serial.println(MQTT_TLS_FINGERPRINT);
#endif
  
  if (WiFi.status() == WL_CONNECTED) {
    SV_PubSubClient.setServer(MQTT_SERVER, MQTT_PORT);
    SV_PubSubClient.setCallback(PubSubClient_callback);
    if (MQTT_WITH_TLS == 'Y') {
      /* MQTT with TLS. */
      if (MQTT_TLS_FINGERPRINT[0] != '\0') {
        /* There is a fingerprint. */
        if (! SV_PacketsClientSecure.setFingerprint(MQTT_TLS_FINGERPRINT)) {
          /* ERROR: Invalide X.509 SHA-1 fingerprint format.. */
#ifdef SVALINN_IN_DEBUG
          Serial.println(F("  ** ERROR: Invalide X.509 SHA-1 fingerprint format."));
#endif
          strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR),
            PSTR("ERROR: Invalide X.509 SHA-1 fingerprint format."),
            SV_MQTT_TLS_FINGERPRINT_MAX_LEN - 1);
          /* Mark device NOT configured. */
          SV_EEPROM_state_write(SV_DEVICE_CONF_STATE_EEPROM_ADDR, 0);
          SV_offline_handle(); /* To reboot. */
        }
      }
      else {
        /* Don’t verify any X.509 certificates since no SHA-1 fingerprint set. */
        SV_PacketsClientSecure.setInsecure();
      }
      /* Nagle=false for not delaying outgoing data in not full packets buf. */
      SV_PacketsClientSecure.setDefaultNoDelay(true);
      /* Writes get automatically flushed. */
      //SV_PacketsClientSecure.setDefaultSync(true);
      SV_PubSubClient.setClient(SV_PacketsClientSecure);
    }
    else {
      SV_PacketsClient.setDefaultNoDelay(true);
      //SV_PacketsClient.setDefaultSync(true);
      SV_PubSubClient.setClient(SV_PacketsClient);
    }
  }
  
  /* Work offline while device is not connected to MQTT server yet */
  while ((! SV_device_is_online) || (! SV_MQTT_functions_no_error)) {
    if (millis() >= SV_OFFLINE_BOOT_TIMEOUT_MS) {
      /* Not connected to MQTT server for long time. */
      SV_offline_handle(); /* To reboot. */
    }
    /* NOTE: connect() can take max time=MQTT_SOCKET_TIMEOUT in a blocking loop. */
    /* connect(unique_id, user, pass,
     *   willTopic, willQos, willRetain, willMessage, cleanSession)
     */
    SV_MQTT_functions_no_error = SV_PubSubClient.connect(
      SV_device_ID, MQTT_USER, MQTT_KEY,
      0, 0, 0, 0, false);
    /* Subscribe to control topic with QoS==SV_MQTT_CTRL_TOPIC_SUB_QOS */
    if (SV_MQTT_functions_no_error) {
      if (! SV_PubSubClient.subscribe(
            (const char*)SV_MQTT_topic_buf, SV_MQTT_CTRL_TOPIC_SUB_QOS))
      {
        SV_MQTT_functions_no_error = false;
      }
    }
#ifdef SVALINN_IN_DEBUG
    if (! SV_MQTT_functions_no_error) {
      Serial.println(F("  ** MQTT ERROR."));
    }
#endif

    SV_device_is_online_check();
  }
  WiFi.setSleepMode(WIFI_MODEM_SLEEP); /* Turn on Wi-Fi power saving. */
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("  Online in (ms): ")); Serial.println(millis());
#endif
}

/* Publish SV_security_state to MQTT server */
void SV_MQTT_publish_security_state(void)
{
  const char* MQTT_TOPIC_PREFIX =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR;
  const char* MQTT_TOPIC_SEC =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_SEC_EEPROM_ADDR;
  snprintf(SV_MQTT_topic_buf, sizeof(SV_MQTT_topic_buf),
    "%s%s", MQTT_TOPIC_PREFIX, MQTT_TOPIC_SEC);
  /* Publish to MQTT server with retained-flag=true and QoS=0 */
  if (! SV_PubSubClient.publish((const char*)SV_MQTT_topic_buf,
          (const uint8_t*)&SV_security_state, sizeof(SV_security_state), true))
  {
    SV_MQTT_functions_no_error = false; /* Error. */
#ifdef SVALINN_IN_DEBUG
    Serial.println(F("# SV_MQTT_publish_security_state(): MQTT ERROR."));
#endif
  }
}

/* Publish SV_alarm_state to MQTT server */
void SV_MQTT_publish_alarm_state(void)
{
  const char* MQTT_TOPIC_PREFIX =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR;
  const char* MQTT_TOPIC_ALARM =
    (const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_ALARM_EEPROM_ADDR;
  snprintf(SV_MQTT_topic_buf, sizeof(SV_MQTT_topic_buf),
    "%s%s", MQTT_TOPIC_PREFIX, MQTT_TOPIC_ALARM);
  /* Publish to MQTT server with retained-flag=true and QoS=0 */
  if (! SV_PubSubClient.publish((const char*)SV_MQTT_topic_buf,
          (const uint8_t*)&SV_alarm_state, sizeof(SV_alarm_state), true))
  {
    SV_MQTT_functions_no_error = false; /* Error. */
#ifdef SVALINN_IN_DEBUG
    Serial.println(F("# SV_MQTT_publish_alarm_state(): MQTT ERROR."));
#endif
  }
}

void SV_MQTT_publish_states(void)
{
  /* To handle the case of ISR triggered while
   * this function working clear the flag first.
   */
  SV_states_scheduled_publish = false;
  /* NOTE: Here you can add a callback function that will run many times
   * depends on SV_STATES_IDLE_PUB_INTERVAL_MS and SV_STATES_ALARM_PUB_INTERVAL_MS
   * such as sending GSM SMS notification or an e-mail.
   */
  SV_MQTT_publish_alarm_state();
  SV_MQTT_publish_security_state();
}

/* Remote security enable/disable */
void SV_remote_security_ctrl(const uint8_t state)
{
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("# SV_remote_alarm_ctrl(): "));
  Serial.println((const char) state);
#endif
  /* Input is verified by SV_valid_security_state() in PubSubClient_callback(). */
  SV_security_ctrl(state);
  /* Read sensors state, no "CHANGE interruption" is possible
   * if a door(s) open while SV_security_state value is SV_SECURITY_STATE_OFF.
   */
  SV_capture_and_act();
  SV_states_scheduled_publish = true;
}

/* Remote alarm signal output control. */
void SV_remote_alarm_ctrl(const uint8_t state)
{
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("# SV_remote_alarm_ctrl(): "));
  Serial.println((const char) state);
#endif
  /* Allow only SV_ALARM_STATE_OFF and SV_ALARM_STATE_REMOTE signals. */
  if ((state == SV_ALARM_STATE_OFF)
   || (state == SV_ALARM_STATE_REMOTE))
  {
    /* To never turn-off alarm when sensors read insecurity,
     * To never replace any active state with turned-on remotly state.
     */
    if ((state == SV_ALARM_STATE_OFF)
     && (get_sensors_states() == SV_ALARM_STATE_OFF))
    {
      SV_alarm_ctrl(SV_ALARM_STATE_OFF);
    }
    else if ((state == SV_ALARM_STATE_REMOTE)
          && (SV_alarm_state == SV_ALARM_STATE_OFF))
    {
      SV_alarm_ctrl(SV_ALARM_STATE_REMOTE);
    }
    SV_capture_and_act();
    SV_states_scheduled_publish = true;
  }
}

void PubSubClient_callback(const char* topic,
                           const uint8_t* payload,
                           const unsigned int payload_len)
{
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("# PubSubClient_callback(): Topic '"));
  Serial.print(topic);
  Serial.print(F("' - payload len: "));
  Serial.println(payload_len);
#endif
  if (payload_len == 1) {
    if (payload[0] == SV_FORCE_PUBLISH_STATES) {
      /* Force publish SV_security_state and SV_alarm_state. */
      SV_capture_and_act();
      SV_states_scheduled_publish = true;
    }
    else if (SV_valid_security_state(payload[0])) {
      /* Control Security status. */
      SV_remote_security_ctrl(payload[0]);
    }
    else if (SV_valid_alarm_state(payload[0])) {
      /* Control alarm signal output. */
      SV_remote_alarm_ctrl(payload[0]);
    }
  }
}

void SV_MQTT_local_loop(void)
{
  yield();
  if (! SV_PubSubClient.loop()) {
    SV_MQTT_functions_no_error = false;
#ifdef SVALINN_IN_DEBUG
    Serial.println(F("# SV_MQTT_local_loop(): MQTT ERROR."));
#endif
  }
  yield();
}

void SV_device_is_online_check(void)
{
  yield();
  SV_device_is_online =
    (WiFi.status() == WL_CONNECTED)
    && SV_PubSubClient.connected()
    && SV_MQTT_functions_no_error;
#ifdef SVALINN_IN_DEBUG
  if (! SV_device_is_online) {
    Serial.print(F("# SV_device_is_online_check(): "));
    Serial.println(SV_device_is_online);
    Serial.print(F("  WiFi.status() = "));
    Serial.println((WiFi.status() == WL_CONNECTED));
    Serial.print(F("  SV_PubSubClient.connected() = "));
    Serial.println(SV_PubSubClient.connected());
    Serial.print(F("  SV_MQTT_functions_no_error = "));
    Serial.println(SV_MQTT_functions_no_error);
  }
#endif
  yield();
}

/* Reboot if device got offline or MQTT function failed */
void SV_offline_handle(void)
{
  if (! SV_device_is_online) {
#ifdef SVALINN_IN_DEBUG
    Serial.println(F("# SV_offline_handle():"));
    SV_debug_dump();
    Serial.println(F("  ** ESP.restart()"));
    Serial.flush();
#endif
    delay(50);
    ESP.restart();
  }
}

#ifdef SVALINN_IN_DEBUG
void SV_debug_dump(void)
{
  Serial.print(F("# SV_debug_dump(): Uptime (m): "));
  Serial.print(millis()/1000.0/60.0);
  Serial.print(F("  -  Free heap kB: "));
  Serial.println(system_get_free_heap_size()/1024.0);
  if ((SV_alarm_state != SV_ALARM_STATE_OFF)
   || (! SV_device_is_online)
   || (! SV_PIR_is_initialized))
  {
    Serial.print(F("  SV_security_state == "));
    Serial.println((const char) SV_security_state);
    Serial.print(F("  SV_alarm_state == "));
    Serial.println((const char) SV_alarm_state);
    Serial.print(F("  SV_PIR_is_initialized == "));
    Serial.println(SV_PIR_is_initialized);
    Serial.print(F("  SV_states_scheduled_publish == "));
    Serial.println(SV_states_scheduled_publish);
  }
}
#endif

/* Handle re-configuring the device mark after pressing
 * the ESP8266 RESET button "SV_RESET_CONF_COUNT times"
 * Within the timeout delay SV_RESET_BUTTON_TIMEOUT_MS.
 */
void SV_reset_button_handle(void)
{
  if (system_get_rst_info()->reason == REASON_EXT_SYS_RST) {
#ifdef SVALINN_IN_DEBUG
    Serial.print(F("# SV_reset_button_handle(): "));
    Serial.println(SV_EEPROM_state_read(SV_RESET_COUNT_EEPROM_ADDR));
#endif
    /* After pressing RESET button SV_RESET_CONF_COUNT times or more */
    if (SV_EEPROM_state_read(SV_RESET_COUNT_EEPROM_ADDR) >= (SV_RESET_CONF_COUNT - 1)) {
      /* Mark device NOT configured. */
      SV_EEPROM_state_write(SV_DEVICE_CONF_STATE_EEPROM_ADDR, 'N');
    }
    else if (SV_EEPROM_state_read(SV_DEVICE_CONF_STATE_EEPROM_ADDR) == 'Y') {
      SV_EEPROM_state_write(SV_RESET_COUNT_EEPROM_ADDR,
        SV_EEPROM_state_read(SV_RESET_COUNT_EEPROM_ADDR) + 1);
      /* A timeout delay loop, to abort the device configuration reset. */
      while (millis() <= SV_RESET_BUTTON_TIMEOUT_MS) {
        delay(50);
      }
    }
  }
  
  SV_EEPROM_state_write(SV_RESET_COUNT_EEPROM_ADDR, 0); /* Reset the counter. */
}

/* Configure the device states, Wi-Fi and MQTT info,
 * Returns 'true' if configuration entering succeed, 'false' in case of an issue.
 */
bool SV_configure_device(void)
{
#ifdef SVALINN_IN_DEBUG
  Serial.println(F("# SV_configure_device()"));
#endif
  uint16_t MQTT_port;
  char tmp_str_buf[16];
  WiFiManager conf_WiFiManager;

#ifndef SVALINN_IN_DEBUG
  conf_WiFiManager.setDebugOutput(false);
#endif
  conf_WiFiManager.resetSettings();
  /* conf_WiFiManager.setConfigPortalTimeout(SV_OFFLINE_BOOT_TIMEOUT_MS / 1000); */
  delay(100);

  /* NOTE: The first (1st) run of the device detection, and anti data format corruption. */
  if ((! SV_valid_security_state(SV_EEPROM_state_read(SV_SECURITY_STATE_EEPROM_ADDR)))
   || (! SV_valid_alarm_state(SV_EEPROM_state_read(SV_ALARM_STATE_EEPROM_ADDR)))
   || (EEPROM.read(SV_MQTT_SERVER_EEPROM_ADDR + SV_MQTT_SERVER_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_USER_EEPROM_ADDR + SV_MQTT_USER_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_KEY_EEPROM_ADDR + SV_MQTT_KEY_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR + SV_MQTT_TLS_FINGERPRINT_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR + SV_MQTT_TOPICS_PREFIX_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_TOPIC_CTRL_EEPROM_ADDR + SV_MQTT_TOPIC_CTRL_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_TOPIC_SEC_EEPROM_ADDR + SV_MQTT_TOPIC_SEC_MAX_LEN - 1) != '\0')
   || (EEPROM.read(SV_MQTT_TOPIC_ALARM_EEPROM_ADDR + SV_MQTT_TOPIC_ALARM_MAX_LEN - 1) != '\0'))
  {
#ifdef SVALINN_IN_DEBUG
    Serial.println(F("  ** First run of SVALINN."));
#endif
    EEPROM.write(SV_SECURITY_STATE_EEPROM_ADDR, SV_SECURITY_STATE_OFF);
    EEPROM.write(SV_ALARM_STATE_EEPROM_ADDR, SV_ALARM_STATE_OFF);
    /* Zero out the EEPROM MQTT settings. */
    for (size_t addr = SV_MQTT_EEPROM_BEGIN_ADDR; addr <= SV_MQTT_EEPROM_END_ADDR; addr++) {
      EEPROM.write(addr, '\0');
    }
    /* Create default configuration.
     * NOTE: Don't count the zero ending in length.
     */
    EEPROM.write(SV_MQTT_PORT_EEPROM_ADDR, (8883 >> 8));
    EEPROM.write(SV_MQTT_PORT_EEPROM_ADDR + 1, (8883 & 0xFF));
    EEPROM.write(SV_MQTT_WITH_TLS_EEPROM_ADDR, 'Y');
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_SERVER_EEPROM_ADDR),
      PSTR("io.adafruit.com"), SV_MQTT_SERVER_MAX_LEN - 1);
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_USER_EEPROM_ADDR),
      PSTR("USER"), SV_MQTT_USER_MAX_LEN - 1);
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_KEY_EEPROM_ADDR),
      PSTR("KEY"), SV_MQTT_KEY_MAX_LEN - 1);
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR),
      PSTR("77:00:54:2D:DA:E7:D8:03:27:31:23:99:EB:27:DB:CB:A5:4C:57:18"),
      SV_MQTT_TLS_FINGERPRINT_MAX_LEN - 1);
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR),
      PSTR("USER/feeds/"), SV_MQTT_TOPICS_PREFIX_MAX_LEN - 1);
    snprintf(tmp_str_buf, sizeof(tmp_str_buf), "%s-%02x%02x",
      PSTR("sv-ctr"), ((chip_ID >> 8) & 0XFF), (chip_ID & 0XFF));
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_CTRL_EEPROM_ADDR),
      tmp_str_buf, SV_MQTT_TOPIC_CTRL_MAX_LEN - 1);
    snprintf(tmp_str_buf, sizeof(tmp_str_buf), "%s-%02x%02x",
      PSTR("sv-s"), ((chip_ID >> 8) & 0XFF), (chip_ID & 0XFF));
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_SEC_EEPROM_ADDR),
      tmp_str_buf, SV_MQTT_TOPIC_SEC_MAX_LEN - 1);
    snprintf(tmp_str_buf, sizeof(tmp_str_buf), "%s-%02x%02x",
      PSTR("sv-a"), ((chip_ID >> 8) & 0XFF), (chip_ID & 0XFF));
    strncpy_P(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_ALARM_EEPROM_ADDR),
      tmp_str_buf, SV_MQTT_TOPIC_ALARM_MAX_LEN - 1);
  }

  /* To make web interface with current configuration in form inputs.
   * NOTE: Don't count the zero ending in length.
   */
  WiFiManagerParameter MQTT_SERVER_WiFiManagerParameter(
    "mqtt_server", PSTR("MQTT Server"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_SERVER_EEPROM_ADDR),
    SV_MQTT_SERVER_MAX_LEN - 1);
  snprintf(tmp_str_buf, sizeof(tmp_str_buf), "%" PRIu16,
    (((uint16_t)EEPROM.read(SV_MQTT_PORT_EEPROM_ADDR) << 8)
    | EEPROM.read(SV_MQTT_PORT_EEPROM_ADDR + 1)));
  WiFiManagerParameter MQTT_PORT_WiFiManagerParameter(
    "mqtt_port", PSTR("MQTT Port"),
    tmp_str_buf, sizeof(tmp_str_buf));
  WiFiManagerParameter MQTT_USER_WiFiManagerParameter(
    "mqtt_user", PSTR("MQTT Username"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_USER_EEPROM_ADDR),
    SV_MQTT_USER_MAX_LEN - 1);
  WiFiManagerParameter MQTT_KEY_WiFiManagerParameter(
    "mqtt_key", PSTR("MQTT Key"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_KEY_EEPROM_ADDR),
    SV_MQTT_KEY_MAX_LEN - 1);
  snprintf(tmp_str_buf, sizeof(tmp_str_buf),
    "%c", EEPROM.read(SV_MQTT_WITH_TLS_EEPROM_ADDR));
  WiFiManagerParameter MQTT_WITH_TLS_WiFiManagerParameter(
    "mqtt_with_tls", PSTR("Use TLS/SSL? Y or N"),
    tmp_str_buf, SV_MQTT_WITH_TLS_MAX_LEN);
  WiFiManagerParameter MQTT_TLS_FINGERPRINT_WiFiManagerParameter(
    "mqtt_tls_fingerprint", PSTR("59 char X.509 SHA-1 fingerprint"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR),
    SV_MQTT_TLS_FINGERPRINT_MAX_LEN -1);
  WiFiManagerParameter MQTT_TOPIC_PREFIX_WiFiManagerParameter(
    "mqtt_topics_prefix", PSTR("MQTT topics prefix"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR),
    SV_MQTT_TOPICS_PREFIX_MAX_LEN - 1);
  WiFiManagerParameter MQTT_TOPIC_CTRL_WiFiManagerParameter(
    "mqtt_topic_ctrl", PSTR("MQTT control topic suffix"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_CTRL_EEPROM_ADDR),
    SV_MQTT_TOPIC_CTRL_MAX_LEN - 1);
  WiFiManagerParameter MQTT_TOPIC_SEC_WiFiManagerParameter(
    "mqtt_topic_sec", PSTR("MQTT security topic suffix"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_SEC_EEPROM_ADDR),
    SV_MQTT_TOPIC_SEC_MAX_LEN - 1);
  WiFiManagerParameter MQTT_TOPIC_ALARM_WiFiManagerParameter(
    "mqtt_topic_alarm", PSTR("MQTT alarm topic suffix"),
    ((const char*)EEPROM.getConstDataPtr() + SV_MQTT_TOPIC_ALARM_EEPROM_ADDR),
    SV_MQTT_TOPIC_ALARM_MAX_LEN - 1);

  conf_WiFiManager.setConnectTimeout(SV_OFFLINE_BOOT_TIMEOUT_MS / 1000);
  conf_WiFiManager.addParameter(&MQTT_SERVER_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_PORT_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_USER_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_KEY_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_WITH_TLS_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_TLS_FINGERPRINT_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_TOPIC_PREFIX_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_TOPIC_CTRL_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_TOPIC_SEC_WiFiManagerParameter);
  conf_WiFiManager.addParameter(&MQTT_TOPIC_ALARM_WiFiManagerParameter);

  SV_LED_indicator_ticker.attach_ms(1000, LED_indicator_ctrl);
  if (conf_WiFiManager.startConfigPortal("Svalinn IoT Alarm")) {
    SV_LED_indicator_ticker.detach();
    /* Write inputs from web interface to configuration memory */
    if (((uint8_t)MQTT_WITH_TLS_WiFiManagerParameter.getValue()[0] == 'Y')
     || ((uint8_t)MQTT_WITH_TLS_WiFiManagerParameter.getValue()[0] == 'y')
     || ((uint8_t)MQTT_WITH_TLS_WiFiManagerParameter.getValue()[0] == 'T')
     || ((uint8_t)MQTT_WITH_TLS_WiFiManagerParameter.getValue()[0] == 't')
     || ((uint8_t)MQTT_WITH_TLS_WiFiManagerParameter.getValue()[0] == '1'))
    {
      EEPROM.write(SV_MQTT_WITH_TLS_EEPROM_ADDR, 'Y');
    }
    else {
      EEPROM.write(SV_MQTT_WITH_TLS_EEPROM_ADDR, 'N');
    }
    MQTT_port = (uint16_t)atoi(MQTT_PORT_WiFiManagerParameter.getValue());
    EEPROM.write(SV_MQTT_PORT_EEPROM_ADDR, (uint8_t)(MQTT_port >> 8));
    EEPROM.write(SV_MQTT_PORT_EEPROM_ADDR + 1, (uint8_t)(MQTT_port & 0xFF));
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_SERVER_EEPROM_ADDR),
      MQTT_SERVER_WiFiManagerParameter.getValue(),
      SV_MQTT_SERVER_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_USER_EEPROM_ADDR),
      MQTT_USER_WiFiManagerParameter.getValue(),
      SV_MQTT_USER_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_KEY_EEPROM_ADDR),
      MQTT_KEY_WiFiManagerParameter.getValue(),
      SV_MQTT_KEY_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_TLS_FINGERPRINT_EEPROM_ADDR),
        MQTT_TLS_FINGERPRINT_WiFiManagerParameter.getValue(),
        SV_MQTT_TLS_FINGERPRINT_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPICS_PREFIX_EEPROM_ADDR),
      MQTT_TOPIC_PREFIX_WiFiManagerParameter.getValue(),
      SV_MQTT_TOPICS_PREFIX_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_CTRL_EEPROM_ADDR),
      MQTT_TOPIC_CTRL_WiFiManagerParameter.getValue(),
      SV_MQTT_TOPIC_CTRL_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_SEC_EEPROM_ADDR),
      MQTT_TOPIC_SEC_WiFiManagerParameter.getValue(),
      SV_MQTT_TOPIC_SEC_MAX_LEN - 1);
    strncpy(((char*)EEPROM.getDataPtr() + SV_MQTT_TOPIC_ALARM_EEPROM_ADDR),
      MQTT_TOPIC_ALARM_WiFiManagerParameter.getValue(),
      SV_MQTT_TOPIC_ALARM_MAX_LEN - 1);
    
    /* Mark device configured;
     * This will also commit() the EEPROM object buffer in RAM to flash memory.
     */
    EEPROM.write(SV_DEVICE_CONF_STATE_EEPROM_ADDR, 'Y');
    return (EEPROM.commit() ? true : false); /* Configuration entry done. */
  }
  
  SV_LED_indicator_ticker.detach();
  /* Mark device NOT configured;
   * This will also commit() the EEPROM object buffer in RAM to flash memory.
   */
  EEPROM.write(SV_DEVICE_CONF_STATE_EEPROM_ADDR, 'N');
  EEPROM.commit();
  return false;
}

void SV_boot(void)
{
  SV_init_pins();
  
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("\n\n\n# SV_boot(): Uptime (ms): "));
  Serial.println(millis());
  Serial.print(F("  Reset Reason: "));
  Serial.println(ESP.getResetReason());
#endif
  
  /* Make SV_device_ID, example: SV0100266DF800164020 */
  chip_ID = ESP.getChipId();
  mem_ID  = ESP.getFlashChipId();
  snprintf(SV_device_ID, sizeof(SV_device_ID),
    PSTR("SV%02X%02X%02X%02X%02X%02X%02X%02X%02X"),
    SV_version[0],
    ((chip_ID >> 24) & 0XFF), ((chip_ID >> 16) & 0XFF),
    ((chip_ID >> 8) & 0XFF), (chip_ID & 0XFF),
    ((mem_ID >> 24) & 0XFF), ((mem_ID >> 16) & 0XFF),
    ((mem_ID >> 8) & 0XFF), (mem_ID & 0XFF));
  
  /* Print ID and MAC, only in normal or by reset button boot. */
  if ((system_get_rst_info()->reason == REASON_DEFAULT_RST)
   || (system_get_rst_info()->reason == REASON_EXT_SYS_RST))
  {
    Serial.print(F("\n\nsvalinn-firmware-esp8266 v"));
    Serial.print(SV_version[0]);
    Serial.print(F("."));
    Serial.print(SV_version[1]);
    Serial.print(F("."));
    Serial.println(SV_version[2]);
    Serial.print(F("Device ID: "));
    Serial.println(SV_device_ID);
    Serial.print(F("Device MAC: "));
    Serial.println(WiFi.macAddress());
  }

  EEPROM.begin(SV_EEPROM_SIZE);
  /* Handle re-configuring the device mark after pressing RESET button 5 times */
  SV_reset_button_handle();
  /* Configure the device if it's not marked configured */
  while ((SV_EEPROM_state_read(SV_DEVICE_CONF_STATE_EEPROM_ADDR) != 'Y')
      || (! SV_valid_security_state(SV_EEPROM_state_read(SV_SECURITY_STATE_EEPROM_ADDR)))
      || (! SV_valid_alarm_state(SV_EEPROM_state_read(SV_ALARM_STATE_EEPROM_ADDR))))
  {
    if (SV_configure_device()) {
      EEPROM.end();
      Serial.flush();
      delay(100);
      ESP.restart();
    }
  }

  /* Get & apply status from flash memory */
  SV_security_ctrl(SV_EEPROM_state_read(SV_SECURITY_STATE_EEPROM_ADDR));
  SV_alarm_ctrl(SV_EEPROM_state_read(SV_ALARM_STATE_EEPROM_ADDR));
  SV_init_sensors();
#ifdef SVALINN_IN_DEBUG
  SV_debug_dump();
#endif

  SV_LED_indicator_ticker.attach_ms(500, LED_indicator_ctrl);
  SV_WiFi_connect();
  SV_MQTT_connect();
  SV_LED_indicator_ticker.detach();
  /* NOTE: From here, The system is Wi-Fi and MQTT connected without errors. */
}

void SV_loop(void)
{
#ifdef SVALINN_IN_DEBUG
  Serial.print(F("# SV_loop():"));
  SV_debug_dump();
#endif

  SV_device_is_online_check();
  SV_offline_handle(); /* Reboot if device is offline or got MQTT error. */
  
  /* Upload states for monitoring */
  if (SV_states_scheduled_publish || (millis() >= SV_states_next_publish_time)) {
    SV_MQTT_publish_states(); /* To upload states */
    if ((SV_alarm_state == SV_ALARM_STATE_OFF)
     || (SV_alarm_state == SV_ALARM_STATE_REMOTE))
    {
      SV_states_next_publish_time = millis() + SV_STATES_IDLE_PUB_INTERVAL_MS;
    }
    else {
      SV_states_next_publish_time = millis() + SV_STATES_ALARM_PUB_INTERVAL_MS;
    }
#ifdef SVALINN_IN_DEBUG
    SV_debug_dump();
#endif
    /* Avoid millis() rollover issues */
    if (millis() >= SV_LONG_UPTIME_TIMEOUT_MS) {
      delay(50);
      ESP.restart();
    }
  }
  
  SV_MQTT_local_loop();
  LED_indicator_ctrl();
}


void setup()
{
  Serial.begin(115200);
  SV_boot();
#ifndef SVALINN_IN_DEBUG
  Serial.flush();
  Serial.end();
#endif
}

void loop()
{
  SV_loop();
  
  /* For power saving, this loop ends after 1s delay, or interruption by sensors.
   * And also will result an MQTT subscriptions latency.
   */
  for (volatile uint8_t i = 0; (i < 10) && (! SV_states_scheduled_publish) ; i++) {
    delay(50);
    delay(50);
  }
}

/* svalinn-firmware-esp8266 end. */
