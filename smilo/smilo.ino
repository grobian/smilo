/*
 * SMILO - SMall Internal Lights Out manager
 *
 * A microcontroller approach to monitoring a serial port and control
 * the power and reset buttons.
 *
 * The name SMILO is a wink to products such as iLO, ALOM, ILOM, iDRAC
 * and BMC, where obviously the capabilities of SMILO are much less, and
 * integration with he host computer is almost nihil.  The main features
 * of SMILO are Serial-over-LAN (SoL) and power and reset button
 * support.
 *
 * SMILO is currently built on top of an OLIMEX ESP32-EVB which contains
 * all the components one needs: LAN-port, 2x relais, uart to connect
 * pins to the serial port of the host board.
 *
 * This program is based on examples:
 * - WiFiTelnetToSerial -- Copyright (c) 2017 Hristo Gochkov
 * - ETH_LAN8720
 * - eeprom_write
 * - HttpBasicAuth
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* TODO:
 * - find GPIO pin for input (34 to 39?) to connect 3.3V from mainboard
 *   to, this could be from the ATX power pin, or perhaps the power LED
 *   pin, provided it emits at most 3.3V (measure first).  Then that
 *   input (digitalRead) could be used to report the current power state
 *   of the host board.  Second thought: there is a 3.3V pin on the COM1
 *   port of the mainboard that we can likely use safely/easily.
 */

/* Connections/wires to be attached to OLIMEX ESP32-EVB
 *
 * The "console" feature is based on the serial port connection from the
 * host, usually the COM1 port.  The COM1 serial header has a standard
 * layout, and three wires are necessary to connect, RX, TX and GND.
 * These three wires are connected to the UEXT1 interface to attach the
 * first UART on the EVB.  Note that the RX and TX wires have RX on one
 * end, and TX on the other, e.g. do not connect UEXT1 RX to COM1 RX
 * pin, but make sure the output of the one, is the input for the other.
 *
 *                     UEXT1                  COM1
 *
 *              ...... ...RX.......   ..RX..
 *              :    : :          :   :    :
 *              :  +-----------+  :   :  +-----------+
 *              :  | 1 2 o o o |   \ /   | 3 o o o   |
 *              :  | o 3 o o o |    X    | o 2 1 o o |
 *              :  +----   ----+   / \   +-----------+
 *              :      :..TX......:   :..TX..: :
 *              :.............GND..............:
 *
 *
 * The power and reset "buttons" are actual relais on on the EVB.  So
 * instead of connecting a GPIO directly to the power and reset pins of
 * the motherboard, a real button -- like on an ATX case, is emulated.
 * This way, the power voltage or which pins need to be used is
 * irrelevant.  The wires from smilo can also be connected together with
 * the ATX case buttons, such that real physical control over the power
 * and reset buttons is not lost.
 * The two relais on the EVB are positioned next to the RJ45 ethernet
 * socket.  The blue connectors allow to connect any wire, and tighten
 * them with screws.  Per relais, three of such screws are visible, only
 * two of them are used -- those that are labeled as "open" (as opposed
 * to "closed"), NO1 and NO2, and the common pins in the middle COM1 and
 * COM2.  The connections to your mainboard differ per board, but in
 * general you'll find a block of pins labelled something like "panel"
 * having pins for "PWRBTN", "RESET", "PWR, "RST" and the like.  Note
 * that "PWRLED", "PLED" or similar is something else, and you should
 * not use those here.
 *
 *          +====+ +-----+
 *          | ETH| | RST | o
 *          |____| +-----+ o \                      o o   PWRLED
 *                 +-----+ o --===== RST ====,      o o   HDDLED   main
 *      EVB        | PWR | o                 '===== o o   RESET    board
 *                 +-----+ o --===== PWR ========== o o   PWRBTN
 *                         o /                      o
 */

#include <ETH.h>
#include <EEPROM.h>
#include <WebServer.h>

/* {{{ Ethernet configuration */
/**
 * Ethernet settings taken from ETH_LAN8720 example.  All of these are
 * given as argument to the ETH.begin() function, and the documentation
 * and settings are retained here just in case a different (or
 * self-composed) board is used to ease porting in that scenario.
 * The setup used here works with the OLIMEX ESP32-EVB.
 */

/*  ETH_CLOCK_GPIO0_IN   - default: external clock from crystal oscillator
 *  ETH_CLOCK_GPIO0_OUT  - 50MHz clock from internal APLL output on
 *                         GPIO0 - possibly an inverter is needed for LAN8720
 *  ETH_CLOCK_GPIO16_OUT - 50MHz clock from internal APLL output on
 *                         GPIO16 - possibly an inverter is needed for LAN8720
 *  ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted
 *                         output on GPIO17 - tested with LAN8720 */
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
/* Pin# of the enable signal for the external crystal oscillator (-1 to
 * disable for internal APLL source) */
#define ETH_POWER_PIN   -1
/* Type of the Ethernet PHY (LAN8720 or TLK110) */
#define ETH_TYPE        ETH_PHY_LAN8720
/* I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110) */
#define ETH_ADDR        0
/* Pin# of the I²C clock signal for the Ethernet PHY */
#define ETH_MDC_PIN     23
/* Pin# of the I²C IO signal for the Ethernet PHY */
#define ETH_MDIO_PIN    18
/* }}} */

/* {{{ EEPROM configuration */
/* size we assume the EEPROM is in bytes */
#define EEPROM_SIZE     (4 * 1024)
#define EEPROM_MAGIC    "SIL0"
/* }}} */

/* {{{ Power relays configuration */
/* pins the relays are connected to */
#define RELAY1_PIN      32
#define RELAY2_PIN      33
/* convenience aliases, unlikely one would want to change this, the
 * board is labelled with REL1 and REL2, but possible if one wants
 * different wiring */
#define POWER_PIN       RELAY1_PIN
#define RESET_PIN       RELAY2_PIN
/* }}} */

/* {{{ Telnet clients configuration */
/* how many clients can be connected at the same time to the console */
#define MAX_SRV_CLIENTS 8

void clients_console_write_bytes(char * s, size_t len);
/* }}} */

/* {{{ global state */
/* whether the ethernet stack is initialised and an IP address is
 * configured, e.g. whether we are ready to accept connections */
static bool           server_ready = false;
/* console history buffer, used to show current console state (or an
 * approximation of it) via the web-endpoint or to populate the console
 * for connecting clients via telnet */
static uint8_t        histbuf[8192];
static size_t         histbuflen = 0;
static size_t         histbufpos = 0;
/* timer var used to timeout stale connections */
static unsigned long  lasttick = 0;
/* }}} */

/* {{{ EEPROM */
/**
 * The EEPROM is used to store some config persistently.  We distinguish
 * a couple of types (eeprom_vartype) that we can store, and a bunch of
 * settings (eeprom_var).  Because there is no point in which we can see
 * we need to store hundreds of vars, each value is prefixed with a
 * single byte that is divided into the type and which var is
 * represented.  The first 3 bits (8 values) are used for the type, the
 * remaining 5 bits (32 values) are used for the var.
 * The settings are all laid out back-to-back in the EEPROM space.  A
 * prefix byte with all zeros denotes the end of the list.
 * Hence when updating values, in particular strings, the entire
 * value space needs to be rewritten to ensure values are back-to-back
 * again.  The order in which the settings appear is not important,
 * updates therefore can move values to the end of the list.
 */

typedef enum eeprom_var {
  VAR_HOSTNAME = 1,
  VAR_CYCLEMS,
  VAR_USERNAME,
  VAR_PASSWORD
#define VAR_FIRST VAR_HOSTNAME
#define VAR_LAST  VAR_CYCLEMS
} eeprom_var;

typedef enum eeprom_type {
  VARTPE_STR   = 1,   /* 4-byte int + bytes */
  VARTPE_INT   = 2    /* 4-byte int */
} eeprom_vartype;

static struct {
  eeprom_var      var;
  eeprom_vartype  type;
  void           *defval;
} eeprom_typemap[] = {
  { (eeprom_var)0, (eeprom_vartype)0, NULL }, /* unset, slot 0 */
  { VAR_HOSTNAME,    VARTPE_STR,   (void *)"smilo" },
  { VAR_CYCLEMS,     VARTPE_INT,   (void *)200     },
  { VAR_USERNAME,    VARTPE_STR,   (void *)"ADMIN" },
  { VAR_PASSWORD,    VARTPE_STR,   (void *)"ADMIN" }
};

void
eeprom_wipe()
{
  int addr = 0;

  Serial.printf("Initialising EEPROM (size=%u bytes)\n", EEPROM_SIZE);
  EEPROM.writeBytes(0, EEPROM_MAGIC, 4);
  addr += 4;
  for (; addr < EEPROM_SIZE; addr++)
    EEPROM.writeByte(addr, 0);
  EEPROM.commit();
}

char _eeprom_var_retbuf[256];
void *
eeprom_get_var(eeprom_var var)
{
  int      addr = sizeof(EEPROM_MAGIC) - 1;
  uint32_t valtype;
  uint32_t valkind;
  uint32_t strvallen;
  uint32_t len;
  void    *retval = NULL;

  /* walk through EEPROM space for as long as we don't find the sentinel
   * or the end of EEPROM space */
  while (addr < EEPROM_SIZE) {
    valtype = EEPROM.readByte(addr);
    addr++;

    /* end of list, stop */
    if (valtype == 0)
      break;

    valkind = valtype & 31;  /* right 5 bits */
    valtype >>= 5;           /* left 3 bits */

    switch (valkind) {
      case VARTPE_STR:
        strvallen = EEPROM.readUInt(addr);
        addr += 4;
        /* truncate, string if it is too long */
        len = strvallen;
        if (len > sizeof(_eeprom_var_retbuf) - 1)
          len = sizeof(_eeprom_var_retbuf) - 1;
        EEPROM.readBytes(addr, (void *)_eeprom_var_retbuf, len);
        addr += strvallen;
        _eeprom_var_retbuf[len] = '\0';
        retval = _eeprom_var_retbuf;
        break;
      case VARTPE_INT:
        len = EEPROM.readUInt(addr);
        addr += 4;
        retval = (void *)len;
        break;
      default:
        /* ignore this value, we don't know how to handle it */
        retval = NULL;
        break;
    }

#if DEBUG
    Serial.printf("EEPROM val %u, type %u, retval %p (%u/%s)\n",
                  valkind, valtype, retval,
                  retval ? (unsigned int)retval : 0, retval);
#endif
    if (valkind == (uint32_t)var)
      break;
  }

  if (retval == NULL)
    retval = eeprom_typemap[var].defval;
  return retval;
}

void
eeprom_set_var(eeprom_var var, void *val)
{
  int      raddr     = sizeof(EEPROM_MAGIC) - 1;
  int      waddr     = sizeof(EEPROM_MAGIC) - 1;
  bool     writedata = false;
  bool     addvalue  = true;
  uint8_t  valtype;
  uint8_t  valkind;
  uint32_t strvallen;
  uint32_t len;

  while (raddr < EEPROM_SIZE) {
    valtype = EEPROM.readByte(raddr);
    raddr++;

    /* end of list, stop */
    if (valtype == 0)
      break;

    valkind = valtype & 31;  /* right 5 bits */
    valtype >>= 5;           /* left 3 bits */

    switch (valkind) {
      case VARTPE_STR:
        strvallen = EEPROM.readUInt(raddr);

        if (valtype == var) {
          /* omit the entire value, we need to write it later, start
           * filling up the gap */
          writedata = true;
          break;
        }

        if (writedata) {
          EEPROM.writeByte(waddr, valtype);
          waddr++;
          EEPROM.writeUInt(waddr, strvallen);
          waddr += 4;
          raddr += 4;
          while (strvallen > 0) {
            EEPROM.writeByte(waddr, EEPROM.readByte(raddr));
            raddr++;
            waddr++;
            strvallen--;
          }
        } else {
          raddr += 4 + strvallen;
          waddr += 1 + 4 + strvallen;
        }

        break;
      case VARTPE_INT:
        len = EEPROM.readUInt(raddr);

        if (valtype == var) {
          /* fixed-width types can be updated in-place */
          EEPROM.writeUInt(waddr, (unsigned int)val);
          addvalue = false;
        }
        raddr += 4;

        if (writedata) {
          EEPROM.writeByte(waddr, valtype);
          waddr++;
          EEPROM.writeUInt(waddr, len);
          waddr += 4;
        } else {
          waddr += 1 + 4;
        }
        break;
      default:
        /* ignore this value, we don't know how to handle it,
         * effectively skip it */
        writedata = true;
        break;
    }
  }

  if (addvalue) {
    valtype = (eeprom_typemap[var].type << 5) | (var & 31);
    EEPROM.writeByte(waddr, valtype);
    waddr++;
    switch (var) {
      case VARTPE_STR:
        strvallen = strlen((char *)val);
        EEPROM.writeUInt(waddr, strvallen);
        waddr += 4;
        EEPROM.writeBytes(waddr, val, (size_t)strvallen);
        waddr += strvallen;
        break;
      case VARTPE_INT:
        EEPROM.writeUInt(waddr, *((int *)val));
        waddr += 4;
        break;
    }

    writedata = true;
  }

  if (writedata) {
    EEPROM.writeByte(waddr, 0);
  }

  EEPROM.commit();
}
/* }}} */

/* {{{ HTTP */
/**
 * HTTP endpoint (non-secured) is used for the following functions:
 * - set/change/clear configuration (stored in EEPROM)
 * - view current console buffer state
 * - trigger power-on
 * - trigger reset
 * All of these are hidden behind Basic auth.
 */

WebServer  http(80);

bool http_handle_auth() {
  static char *user = (char *)eeprom_get_var(VAR_USERNAME);
  static char *pass = (char *)eeprom_get_var(VAR_PASSWORD);

  if (!http.authenticate(user, pass)) {
    http.requestAuthentication(BASIC_AUTH,
                               "SMiLO",
                               "use ADMIN/ADMIN as default values");
    return false;
  }

  return true;
}

#define http_prologue() \
  if (!http_handle_auth()) \
    return

void http_handle_trigger_power() {
  char msg[128];
  size_t msgsiz;

  http_prologue();

  msgsiz = snprintf(msg, sizeof(msg),
                    "[request to press power button from %s:%u]\n",
                    http.client().remoteIP().toString().c_str(),
                    http.client().remotePort());
  clients_console_write_bytes(msg, msgsiz);
  clients_history_write_bytes(msg, msgsiz);

  digitalWrite(POWER_PIN, HIGH);
  delay(200);
  digitalWrite(POWER_PIN, LOW);
  http.send(200, "text/plain", "power button was pressed");
}

void http_handle_hold_power_5s() {
  char msg[128];
  size_t msgsiz;

  http_prologue();

  msgsiz = snprintf(msg, sizeof(msg),
                    "[request to hold power button 5s from %s:%u]\n",
                    http.client().remoteIP().toString().c_str(),
                    http.client().remotePort());
  clients_console_write_bytes(msg, msgsiz);
  clients_history_write_bytes(msg, msgsiz);

  digitalWrite(POWER_PIN, HIGH);
  delay(5000);
  digitalWrite(POWER_PIN, LOW);
  http.send(200, "text/plain", "power button was pressed for 5s");
}

void http_handle_trigger_reset() {
  char msg[128];
  size_t msgsiz;

  http_prologue();

  msgsiz = snprintf(msg, sizeof(msg),
                    "[request to press reset button from %s:%u]\n",
                    http.client().remoteIP().toString().c_str(),
                    http.client().remotePort());
  clients_console_write_bytes(msg, msgsiz);
  clients_history_write_bytes(msg, msgsiz);

  digitalWrite(RESET_PIN, HIGH);
  delay(200);
  digitalWrite(RESET_PIN, LOW);
  http.send(200, "text/plain", "reset button was pressed");
}

void http_handle_config_reset() {
  char msg[128];
  size_t msgsiz;

  http_prologue();

  msgsiz = snprintf(msg, sizeof(msg),
                    "[configuration reset from %s:%u, board rebooting, "
                    "expect connection loss]\n",
                    http.client().remoteIP().toString().c_str(),
                    http.client().remotePort());
  clients_console_write_bytes(msg, msgsiz);
  clients_history_write_bytes(msg, msgsiz);

  http.send(200, "text/plain",
            "configuration reset successful, please wait while board reboots");
  eeprom_wipe();
  delay(2000);
  ESP.restart();
}

void http_handle_root() {
  static char *hostname = (char *)eeprom_get_var(VAR_HOSTNAME);
  size_t drlen = sizeof(histbuf) + 1024;
  char *dataresponse;

  http_prologue();

  dataresponse = (char *)malloc(drlen);
  if (dataresponse == NULL) {
    /* TODO: should we be more forgiving and try without current
     * console buffer?  Questionable if it should help, histbuf
     * currently just is 8K, which really is an issue when that size
     * cannot be allocated here */
    http.send(500, "text/plain", "failed to allocate memory");
    return;
  }

  snprintf(dataresponse, drlen,
           "<html><head><title>SMiLO - %s</title></head>"
           "<body><h1>SMiLO - %s</h1>"
           "<ul>"
           "<li><a href='/trigger/reset'>reset</a></li>"
           "<li><a href='/trigger/power'>power</a></li>"
           "<li><a href='/hold/power'>hold power 5s</a></li>"
           "<li><a href='/conf/reset'>reset configuration to defaults</a></li>"
           "</ul>"
           "<h2>console</h2>"
           "<div style='background-color:LightGray;'><pre>%.*s%.*s</pre></div>"
           "</body>"
           "</html>",
           hostname, hostname,
           (int)(histbuflen - histbufpos), &histbuf[histbufpos],
           (int)histbufpos, histbuf);

  http.send(200, "text/html", dataresponse);
  free(dataresponse);
}
/* }}} */

/* {{{ Telnet client */
typedef enum client_state {
  SCS_FREE = 0,
  SCS_AUTH,
  SCS_MENU,
  SCS_SOL
} client_state;

typedef struct client {
  WiFiClient    connection;
  unsigned long connect_at;
  client_state  state;
} client;

client clients[MAX_SRV_CLIENTS];

WiFiServer server(23);

/* console clients, these are connected via a TCP socket */
void clients_console_write_bytes(char *s, size_t len)
{
  int i;

  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (clients[i].state == SCS_SOL &&
        clients[i].connection.connected())
      clients[i].connection.write((uint8_t *)s, len);
  }
}

void clients_history_write_bytes(char *s, size_t len)
{
  size_t size;
  size_t pos  = 0;

  /* wrap buffer as many times as required */
  do {
    if (histbufpos == sizeof(histbuf))
      histbufpos = 0;
    size = sizeof(histbuf) - histbufpos;
    if (size > len)
      size = len;
    memcpy(&histbuf[histbufpos], s + pos, size);
    histbufpos += size;
    pos        += size;
    len        -= size;
    if (histbuflen < histbufpos)
      histbuflen = histbufpos;
  } while (len > 0);
}
/* }}} */

/* {{{ network stack state change handler
 * this sets up the appropriate things such as network listeners and
 * http endpoints when the stack comes up, closes things down, etc.
 * server_ready global is set from here */
void net_handle_event(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START: {
      char *hostname = (char *)eeprom_get_var(VAR_HOSTNAME);
      Serial.printf("MAC address: %s\n", ETH.macAddress().c_str());
      Serial.printf("Hostname: %s\n", hostname);
      ETH.setHostname(hostname);
    } break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.printf("Ethernet link up: %uMbps, %s duplex\n",
                    ETH.linkSpeed(), ETH.fullDuplex() ? "full" : "half");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("Received address: %s gw %s dns %s\n",
                    ETH.localIP().toString().c_str(),
                    ETH.gatewayIP().toString().c_str(),
                    ETH.dnsIP().toString().c_str());
      Serial.println("Starting telnet server on port 23");
      server.begin();
      server.setNoDelay(true);
      server_ready = true;

      Serial.println("Starting http server on port 80");
      http.on("/",              http_handle_root);
      http.on("/trigger/power", http_handle_trigger_power);
      http.on("/hold/power",    http_handle_hold_power_5s);
      http.on("/trigger/reset", http_handle_trigger_reset);
      http.on("/conf/reset",    http_handle_config_reset);
      http.begin();
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet link down");
      server.stopAll();
      http.stop();
      server_ready = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet removed");
      server.stopAll();
      server.end();
      http.stop();
      http.close();
      server_ready = false;
      break;
    default:
      Serial.printf("unhandled event %u\n", event);
      break;
  }
}
/* }}} */

/* {{{ Arduino setup function
 * called once during startup */
void setup() {
  char eeprom_magic[4];

  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  delay(2000);

  /* check magic for EEPROM, else wipe */
  if (EEPROM.readBytes(0, eeprom_magic, 4) != 4 ||
      memcmp(eeprom_magic, EEPROM_MAGIC, 4) != 0)
    eeprom_wipe();

  Serial.printf("Setting up ethernet %s\n",
                ETH_TYPE == ETH_PHY_LAN8720 ? "LAN8720" : "TLK110");
  WiFi.onEvent(net_handle_event);  /* counter-intuitive: but also for ETH */
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN,
  			    ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

  Serial.println("Setting up serial connection: 115200 baud");
  Serial1.begin(115200, SERIAL_8N1, 36, 4, true);

  Serial.println("Setting up power and reset pins for output");
  pinMode(POWER_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
}
/* }}} */

/* {{{ Arduino loop function
 * this function is called repeatedly, forever, handling of incoming
 * connections and reading/writing data from the connected serial is
 * done from here */
void loop() {
  int i;
  int connected_clients;
  size_t savail;
  WiFiClient newclient;

  if (!server_ready)
    goto snooze;

  /* add new connections, if any */
  if (server.hasClient()) {
    newclient = server.accept();
    if (!newclient) {
      Serial.println("Failed to accept new client, ignoring");
      return;
    }

    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (clients[i].state == SCS_FREE) {
        clients[i].connection = newclient;
        clients[i].connect_at = millis();
        clients[i].state      = SCS_AUTH;
        break;
      }
    }
    if (i == MAX_SRV_CLIENTS) {
      Serial.println("Cannot accept new connection: too many clients");
      newclient.stop();
      return;
    }

    Serial.printf("New connection from %s:%u\n",
                  clients[i].connection.remoteIP().toString().c_str(),
                  clients[i].connection.remotePort());
    clients[i].connection.printf("[successfully connected to %s]\n",
                                 ETH.localIP().toString().c_str());
    /* dump current history buffer */
    clients[i].connection.write(&histbuf[histbufpos], histbuflen - histbufpos);
    clients[i].connection.write(histbuf, histbufpos);
  }

  /* proxy data from uart to clients and/or buffer, read in blocks
   * output usually is whole lines, this purposely doesn't use a loop so
   * we keep responsive doing other things when somehow the uart is
   * spamming like hell here */
  if ((savail = Serial1.available()) > 0) {
    /* unfortunately using clients_history_write_bytes here would
     * require an extra copy, so this bit is an almost copy of that
     * function for the sake of some speed */
    if (histbufpos == sizeof(histbuf))
      histbufpos = 0;
    if (savail > sizeof(histbuf) - histbufpos)
      savail = sizeof(histbuf) - histbufpos;
    savail = Serial1.readBytes(&histbuf[histbufpos], savail);
    clients_console_write_bytes((char *)&histbuf[histbufpos], savail);
    histbufpos += savail;
    if (histbuflen < histbufpos)
      histbuflen = histbufpos;
  }

  connected_clients = 0;
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (clients[i].state != SCS_FREE) {
      if (!clients[i].connection.connected()) {
        Serial.printf("Lost connection from %s:%u after %us\n",
                      clients[i].connection.remoteIP().toString().c_str(),
                      clients[i].connection.remotePort(),
                      (millis() - clients[i].connect_at) / 1000);
        clients[i].connection.stop();
        clients[i].state = SCS_FREE;
      } else {
        /* proxy data from clients to uart, byte by byte since this most
         * likely is what the input is (keyboard) */
        while (clients[i].connection.available()) {
          Serial.println("writing a byte to uart");
          Serial1.write(clients[i].connection.read());
        }

        connected_clients++;
      }
    }
  }

  if (lasttick + 30000 < millis()) {
    if (connected_clients > 0) {
      Serial.printf("Connected clients: %u\n", connected_clients);
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (clients[i].state != SCS_FREE) {
          Serial.printf("[%u] %s:%u for %us\n",
                        i,
                        clients[i].connection.remoteIP().toString().c_str(),
                        clients[i].connection.remotePort(),
                        (millis() - clients[i].connect_at) / 1000);
        }
      }
    }
    lasttick = millis();
  }

  http.handleClient();

  /* trigger re-iteration so existing connections remain responsive */
  if (connected_clients > 0 || savail > 0)
    return;

snooze:
  delay(1000 + (rand() % 250));
  return;
}
/* }}} */

/* vim: set ts=2 sw=2 expandtab foldmethod=marker: */
