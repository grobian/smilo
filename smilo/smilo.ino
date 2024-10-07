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
 * all the components one needs: LAN-port, 2x relais, UART to connect
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
 * - the web-interface needs a lot of love
 * - respond to the USB serial connection, allow things like rebooting,
 *   config reset
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
 *              :  | 1 2 o o 4 |   \ /   | 3 o o o   |
 *              :  | o 3 o o o |    X    | o 2 1 o o |
 *              :  +----   ----+   / \   +----   ----+
 *              :      :..TX......:   :..TX..: :
 *              :.............GND..............:
 *
 *                           4 = input pin for board power (at most 5V)
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
 *                                                  UEXT1 pin 4
 *                                                  |
 *          +====+ +-----+                          :
 *          | ETH| | RST | o                        |
 *          |____| +-----+ o \                      o o   PWRLED
 *                 +-----+ o --===== RST ====,      o o   HDDLED   main
 *      EVB        | PWR | o                 '===== o o   RESET    board
 *                 +-----+ o --===== PWR ========== o o   PWRBTN
 *                         o /                      o
 *
 * 
 * Usage
 * =====
 *
 * Once flashed, SMILO boots and waits for a connection either via
 * telnet (port 23) or http (port 80).  The default credentials are
 * ADMIN/ADMIN, and can be changed via the properties menu.  To reset to
 * the default settings, press and hold BUT1 on the EVB until it resets
 * itself.
 *
 * Telnet
 * ------
 * 
 * Connections via telnet better be via a telnet client, e.g. not nc.
 * SMILO understands a limited part of the telnet protocol, but enough
 * to handle backspaces and do escape processing.
 * At any time while connected to the serial console, one can return
 * back to SMILO's main menu by sending an IP (Interrupt Process) telnet
 * command.
 */

#include <ETH.h>
#include <EEPROM.h>
#include <WebServer.h>

#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif

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

/* UART1 pins */
#define UART1_RX_PIN 36
#define UART1_TX_PIN  4

void clients_console_write_bytes(char * s, size_t len, int id);
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
/* whether we believe the host is powered on */
static bool           host_powered_on = false;

/* pin to get board power from */
#define BOARD_PWR_PIN 17  /* SPI0:CS0, UEXT1 pin 10 */

/* "reset" button */
#define ESP_BUT1_PIN  34
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

/* NOTE: remember that this enum holds the keys for the values stored in
 *       EEPROM, so do only APPEND to this list, as changing the order
 *       will mess up existing configuration! */
typedef enum eeprom_var {
  VAR_PASSWORD = 1,
  VAR_USERNAME,
  VAR_HOSTNAME,
  VAR_CYCLEMS,
  VAR_HOLDMS,
  VAR_BAUD_ON,
  VAR_BAUD_OFF,
  VAR_INVERT
#define VAR_FIRST VAR_PASSWORD
#define VAR_LAST  VAR_INVERT
} eeprom_var;

typedef enum eeprom_type {
  VARTPE_STR   = 1,   /* 4-byte int + bytes */
  VARTPE_INT   = 2,   /* 4-byte int */
  VARTPE_BOL   = 3,   /* 1-byte int */
} eeprom_vartype;

static struct {
  eeprom_var      var;
  const char     *varname;
  eeprom_vartype  type;
  const void     *defval;
} eeprom_typemap[] = {
  { (eeprom_var)0, NULL, (eeprom_vartype)0, NULL }, /* unset, slot 0 */
  { VAR_PASSWORD, "password",               VARTPE_STR, (void *)"ADMIN" },
  { VAR_USERNAME, "username/password",      VARTPE_STR, (void *)"ADMIN" },
  { VAR_HOSTNAME, "hostname",               VARTPE_STR, (void *)"smilo" },
  { VAR_CYCLEMS,  "press time ms",          VARTPE_INT, (void *)200     },
  { VAR_HOLDMS,   "hold time ms",           VARTPE_INT, (void *)5000    },
  { VAR_BAUD_ON,  "COM1 baudrate host on",  VARTPE_INT, (void *)115200  },
  { VAR_BAUD_OFF, "COM1 baudrate host off", VARTPE_INT, (void *)50      },
  { VAR_INVERT,   "COM1 invert mode",       VARTPE_BOL, (void *)1       }
#define MAX_DEFVAL_SIZE  6
#define MAX_VARNAME_SIZE 22
};

void
eeprom_wipe()
{
  EEPROM.writeBytes(0, "WIPE", 4);
  EEPROM.commit();
}

void
eeprom_init()
{
  int addr = 0;

  Serial.printf("Initialising EEPROM (size=%u bytes)\r\n", EEPROM_SIZE);
  EEPROM.writeBytes(0, EEPROM_MAGIC, 4);
  addr += 4;
  for (; addr < EEPROM_SIZE; addr++)
    EEPROM.writeByte(addr, 0);
  EEPROM.commit();
}

char _eeprom_var_retbuf[256];
const void *
eeprom_get_var(eeprom_var var)
{
  int         addr      = sizeof(EEPROM_MAGIC) - 1;
  uint32_t    valtype;
  uint32_t    valkind;
  uint32_t    strvallen;
  uint32_t    len;
  const void *retval    = NULL;

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

    switch (valtype) {
      case VARTPE_STR:
        strvallen = EEPROM.readUInt(addr);
        addr += 4;
        if (valkind == (uint32_t)var) {
          /* truncate, string if it is too long */
          len = strvallen;
          if (len > sizeof(_eeprom_var_retbuf) - 1)
            len = sizeof(_eeprom_var_retbuf) - 1;
          EEPROM.readBytes(addr, (void *)_eeprom_var_retbuf, len);
          _eeprom_var_retbuf[len] = '\0';
          retval = _eeprom_var_retbuf;
        }
        addr += strvallen;
        break;
      case VARTPE_INT:
        if (valkind == (uint32_t)var) {
          len = EEPROM.readUInt(addr);
          retval = (void *)len;
        }
        addr += 4;
        break;
      case VARTPE_BOL:
        if (valkind == (uint32_t)var) {
          retval = (void *)(size_t)EEPROM.readByte(addr);
        }
        addr++;
        break;
      default:
        /* ignore this value, we don't know how to handle it */
        retval = NULL;
        break;
    }

    if (valkind == (uint32_t)var)
      break;
  }

  if (retval == NULL)
    retval = eeprom_typemap[var].defval;
  return retval;
}
#define eeprom_get_var_str(VAR)  (char *)eeprom_get_var(VAR)
#define eeprom_get_var_int(VAR)  (uint32_t)eeprom_get_var(VAR)
#define eeprom_get_var_bol(VAR)  (bool)eeprom_get_var(VAR)

void
eeprom_set_var(eeprom_var var, const void *val)
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

    /* stop as soon as we find something that doesn't make sense */
    if (valkind < VAR_FIRST || valkind > VAR_LAST) {
      writedata = true;
      break;
    }

    switch (valtype) {
      case VARTPE_STR:
        strvallen = EEPROM.readUInt(raddr);
        raddr += 4 + strvallen;

        if (valkind == var) {
          /* omit the entire value, we need to write it later, start
           * filling up the gap */
          writedata = true;
          addvalue  = true;
          break;
        }

        if (writedata) {
          EEPROM.writeByte(waddr, valtype << 5 | valkind);
          waddr++;
          EEPROM.writeUInt(waddr, strvallen);
          waddr += 4;
          raddr -= strvallen;
          while (strvallen > 0) {
            EEPROM.writeByte(waddr, EEPROM.readByte(raddr));
            raddr++;
            waddr++;
            strvallen--;
          }
        } else {
          waddr += 1 + 4 + strvallen;
        }

        break;
      case VARTPE_INT:
        len = EEPROM.readUInt(raddr);
        raddr += 4;

        if (valkind == var) {
          /* fixed-width types can be updated in-place */
          len = (uint32_t)val;
          addvalue = false;
        }

        if (writedata || valkind == var) {
          EEPROM.writeByte(waddr, valtype << 5 | valkind);
          waddr++;
          EEPROM.writeUInt(waddr, len);
          waddr += 4;
        } else {
          waddr += 1 + 4;
        }
        break;
      case VARTPE_BOL:
        len = EEPROM.readByte(raddr);
        raddr++;

        if (valkind == var) {
          /* like int, do in-place */
          len = (uint32_t)val;
          addvalue = false;
        }

        if (writedata || valkind == var) {
          EEPROM.writeByte(waddr, valtype << 5 | valkind);
          waddr++;
          EEPROM.writeByte(waddr, (uint8_t)len);
          waddr++;
        } else {
          waddr += 1 + 1;
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
    valtype = (eeprom_typemap[var].type << 5) | ((int)var & 31);
    EEPROM.writeByte(waddr, valtype);
    waddr++;
    switch (eeprom_typemap[var].type) {
      case VARTPE_STR:
        strvallen = strlen((char *)val);
        EEPROM.writeUInt(waddr, strvallen);
        waddr += 4;
        EEPROM.writeBytes(waddr, val, (size_t)strvallen);
        waddr += strvallen;
        break;
      case VARTPE_INT:
        EEPROM.writeUInt(waddr, (unsigned int)val);
        waddr += 4;
        break;
      case VARTPE_BOL:
        EEPROM.writeByte(waddr, (uint8_t)(size_t)val);
        waddr++;
        break;
    }

    writedata = true;
  }

  if (writedata) {
    EEPROM.writeByte(waddr, 0);
  }

  EEPROM.commit();
}
/* arduino doesn't seem to have ptr_t, use size_t instead */
#define eeprom_set_var_str(VAR,VAL)  eeprom_set_var(VAR,(void *)VAL)
#define eeprom_set_var_int(VAR,VAL)  eeprom_set_var(VAR,(void *)(size_t)VAL)
#define eeprom_set_var_bol(VAR,VAL)  eeprom_set_var(VAR,(void *)(size_t)VAL)
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
  /* cache this statically, because HTTP request will look this up over
   * and over again */
  static char *user = strdup(eeprom_get_var_str(VAR_USERNAME));
  static char *pass = strdup(eeprom_get_var_str(VAR_PASSWORD));

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
  clients_console_write_bytes(msg, msgsiz, -1);
  clients_history_write_bytes(msg, msgsiz);

  digitalWrite(POWER_PIN, HIGH);
  delay(eeprom_get_var_int(VAR_CYCLEMS));
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
  clients_console_write_bytes(msg, msgsiz, -1);
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
  clients_console_write_bytes(msg, msgsiz, -1);
  clients_history_write_bytes(msg, msgsiz);

  digitalWrite(RESET_PIN, HIGH);
  delay(eeprom_get_var_int(VAR_CYCLEMS));
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
  clients_console_write_bytes(msg, msgsiz, -1);
  clients_history_write_bytes(msg, msgsiz);

  http.send(200, "text/plain",
            "configuration reset successful, please wait while board reboots");
  eeprom_wipe();
  delay(200);
  ESP.restart();
}

/* strip unwanted control chars and ANSI escapes from output suitable
 * for HTML
 * this function handles the history buffer wrap */
size_t http_strip_ansi_escape(char  *dest,
                              size_t destsiz,
                              size_t pos,
                              size_t len)
{
  size_t retlen = 0;
  size_t i;
  char   p;
  enum state {
    S_NORM = 0,
    S_ESC,
    S_BRACL,
    S_ARG,
    S_BRACR,
    S_TITLE
  }      curstate = S_NORM;

  /* the idea is that we just copy byte by byte scanning forwards,
   * skipping over the input when we detect an ANSI escape of some sort
   * perhaps less efficient, but we keep a state such that we can
   * evaluate byte by byte, and wrap around the buffer edges easily
   * we don't validate here, the idea is that we assume the input is
   * valid, that is, if we get confused, we kick back to emitting the
   * initial processing state, outputting whatever garbage there
   * currently is
   * ANSI codes:
   * https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797 */

  for (i = 0; i < len; i++) {
    if (retlen == destsiz)
      return retlen;
    p = (char)histbuf[(pos + i) % histbuflen];
    switch (curstate) {
      case S_NORM:
        if (p == '\033') {
          curstate = S_ESC;
          break;
        }
        /* skip unprintable chars */
        if (p < ' ' && p != '\n' && p != '\t')
          break;
        if (p >= 127)
          break;

        dest[retlen++] = p;
        break;
      case S_ESC:
        if (p == '[') {
          curstate = S_BRACL;
          break;
        } else if (p == ']') {
          curstate = S_BRACR;
          break;
        } else {
          curstate = S_NORM;
          i--;  /* re-evaluate */
          break;
        }
        break;
      case S_BRACL:
        curstate = S_ARG;
        if (p == '=' || p == '?')
          break;
        /* fall-through: this is an argument now */
      case S_ARG:
        if (p >= '0' && p <= '9')
          break;
        if (p == ';')
          break;
        /* any letter (non-number or semi-colon) means end of this
         * directive, check if we have to map this directive into
         * something else */
        switch (p) {
          case 'H':
          case 'f':
          case 'E':
          case 'F':
          case 'J':
            dest[retlen++] = '\n';
            break;
        }
        curstate = S_NORM;
        break;
      case S_BRACR:
        curstate = S_TITLE;
        /* fall-through: find the title end */
      case S_TITLE:
        if (p == '\007')
          curstate = S_NORM;
        break;
    }
  }

  return retlen;
}

void http_handle_root() {
  static char *hostname     = strdup(eeprom_get_var_str(VAR_HOSTNAME));
  size_t       drlen        = sizeof(histbuf) + 1024;
  static char *dataresponse = (char *)malloc(drlen);
  size_t       pos;
  size_t       remain;
  size_t       bpos;

  http_prologue();

  if (dataresponse == NULL) {
    /* TODO: should we be more forgiving and try without current
     * console buffer?  Questionable if it should help, histbuf
     * currently just is 8K, which really is an issue when that size
     * cannot be allocated here */
    http.send(500, "text/plain", "failed to allocate memory");
    return;
  }

  bpos = snprintf(dataresponse, drlen,
           "<html><head><title>SMiLO - %s</title></head>"
           "<body><h1>SMiLO - %s</h1>"
           "<p>MAC: %s</p>"
           "<ul>"
           "<li><a href='/trigger/reset'>reset</a></li>"
           "<li><a href='/trigger/power'>power</a></li>"
           "<li><a href='/hold/power'>hold power 5s</a></li>"
           "<li><a href='/conf/reset'>reset configuration to defaults</a></li>"
           "</ul>"
           "<h2>console</h2>"
           "<div style='background-color:LightGray;'><pre>",
           hostname, hostname,
           ETH.macAddress().c_str());

  pos    = clients_history_find_last_screen();
  if (pos < histbufpos)
    remain = histbufpos - pos;
  else
    remain = histbuflen - (pos - histbufpos);

  bpos += http_strip_ansi_escape(dataresponse + bpos,
                                 drlen - bpos,
                                 pos, remain);
  bpos += snprintf(dataresponse + bpos, drlen - bpos,
           "</pre></div>"
           "</body>"
           "</html>");

  http.send(200, "text/html", dataresponse);
}
/* }}} */

/* {{{ Telnet client */
typedef enum client_auth_state {
  SCSA_INIT = 0,
  SCSA_SEND_USER,
  SCSA_RECV_USER,
  SCSA_SEND_PASS,
  SCSA_RECV_PASS,
  SCSA_CHECK_CRED
} client_auth_state;

struct scs_auth {
#define SCSA_BUFMAXLEN 20
  char      username[SCSA_BUFMAXLEN];
  int       usernamelen;
  char      password[SCSA_BUFMAXLEN];
  int       passwordlen;
  int       authtries;
}           auth;

typedef enum client_menu_state {
  SCSM_INIT = 0,
  SCSM_SEND_MAIN,
  SCSM_RECV_MAIN,
  SCSM_SEND_RESET,
  SCSM_SEND_POWER,
  SCSM_SEND_POWER5S
} client_menu_state;

struct scs_menu {
  void *dummy;
};

typedef enum client_prop_state {
  SCSP_INIT = 0,
  SCSP_SEND_PROP,
  SCSP_RECV_PROP,
  SCSP_RECV_VAL,
  SCSP_RECV_PASSCHK
} client_prop_state;

struct scs_prop {
#define SCSP_BUFMAXLEN 64
  char       buf[SCSP_BUFMAXLEN];
  int        buflen;
  char       passcheck[SCSA_BUFMAXLEN];
  int        passchecklen;
  eeprom_var var;
}            prop;

typedef enum client_sol_state {
  SCSS_INIT = 0,
  SCSS_SOL,
  SCSS_BRK
} client_sol_state;

struct scs_sol {
  void *dummy;
};

typedef enum client_state {
  SCS_FREE = 0,
  SCS_STOP,
  SCS_AUTH,
  SCS_MENU,
  SCS_PROP,
  SCS_SOL,
  SCS_INTR
} client_state;

typedef struct client {
  WiFiClient    connection;
  unsigned long connect_at;
  client_state  state;
  int           seq;
  unsigned char echo_replied:1;
  unsigned char localecho_replied:1;
  union {
    struct scs_auth
                auth;
    struct scs_menu
                menu;
    struct scs_prop
                prop;
    struct scs_sol
                sol;
  }             statectx;
} client;

/* console clients, these are connected via a TCP socket */
client clients[MAX_SRV_CLIENTS];

WiFiServer telnet(23);

/* telnet is specified in a bunch of RFCs, main one:
 * https://www.rfc-editor.org/rfc/rfc854
 * follow-ups for features are referenced from here:
 * https://www.iana.org/assignments/telnet-options/telnet-options.xhtml
 */
void clients_console_write_bytes(char *s, size_t len, int id)
{
  int i;
  size_t startb;
  size_t endb;

  for (startb = 0, endb = 0; endb < len; endb++) {
    if (s[endb] == '\n' ||
        s[endb] == 127  ||  /* DEL */
        endb == (len - 1))
    {
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if ((id == i ||
             (id < 0 &&
              clients[i].state == SCS_SOL &&
              (client_sol_state)(clients[i].seq) == SCSS_SOL)) &&
            clients[i].connection.connected())
        {
          if (s[endb] == '\n') {
            clients[i].connection.write((uint8_t *)(&s[startb]),
                                        endb - startb);
            clients[i].connection.write("\r\n");
          } else if (s[endb] == 127) {  /* DEL */
            clients[i].connection.write((uint8_t *)(&s[startb]),
                                        endb - startb);
            clients[i].connection.write('\b');  /* BS */
            clients[i].connection.write(' ');   /* SP */
            clients[i].connection.write('\b');  /* BS */
          } else {
            clients[i].connection.write((uint8_t *)(&s[startb]),
                                        endb + 1 - startb);
          }
        }
      }
      startb = endb + 1;
    }
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

/* return position (or start of buffer) after the last clearscreen */
size_t clients_history_find_last_screen(void)
{
  char  *p;
  char   storebuf[8] = { '\0', '\0', '\0', '\0',  '\0', '\0', '\0', '\0' };
  size_t i;
  size_t clen;

  /* currently, we only search for clrscrn, alternate screen and restore:
   * \033 [ (0|1|2|3)? J   3 or 4 bytes
   * \033 [ ? 1049 (h|l)   8 bytes
   * \033 [ ? 47 l         6 bytes
   * \033 [ H              3 bytes
   * the strategy is complex in favour of returning as fast as possible
   * by searching backwards */

  /* shortcut empty/early buffer case */
  if (histbuflen < 3)
    return 0;

  /* match codes in BUF, at POS, where POS >= 8, checks ESC first, since
   * it should happen less often than things like J, l, h */
#define _match_code(BUF,POS) \
  ( \
    ( \
      BUF[POS - 3] == '\003' && BUF[POS - 2] == '[' && \
      BUF[POS - 1] >= '0'    && BUF[POS - 1] <= '3' && \
      BUF[POS]     == 'J' \
    ) \
    || \
    ( \
      BUF[POS - 2] == '\033' && BUF[POS - 1] == '[' && \
      BUF[POS]     == 'J' \
    ) \
    || \
    ( \
      BUF[POS - 2] == '\033' && BUF[POS - 1] == '[' && \
      BUF[POS]     == 'H' \
    ) \
    || \
    ( \
      BUF[POS - 7] == '\033' && BUF[POS - 6] == '[' && \
      BUF[POS - 5] == '?'    && BUF[POS - 4] == '1' && \
      BUF[POS - 3] == '0'    && BUF[POS - 2] == '4' && \
      BUF[POS - 1] == '9'    && \
      ( \
        BUF[POS] == 'h' || BUF[POS] == 'l' \
      ) \
    ) \
    || \
    ( \
      BUF[POS - 5] == '\033' && BUF[POS - 4] == '[' && \
      BUF[POS - 3] == '?'    && BUF[POS - 2] == '4' && \
      BUF[POS - 1] == '7'    && BUF[POS]     == 'l' \
    ) \
  )
#define match_code(BUF,POS) _match_code(((char *)(BUF)), (POS))

  /* walk through the buffer, ensure there's always 8-byte room for the
   * macro check, therefore, on the tail and edges of the wrap use
   * storebuf to provide the space to the macro */

  /* phase one: the easy bit, scan whatever's there until the front of
   * the buffer */
  for (i = histbufpos; i >= 8; i--)
    if (match_code(histbuf, i - 1))
      return i;

  /* phase two: scan the remainder of the front of the buffer, either
   * with data from the tail (wrap), or zeros when there is none */
  if (histbuflen > histbufpos) {
    for (i = 8; i >= 3; i--) {
      clen = 8 - (i - 1);
      memcpy(&storebuf[clen], histbuf, i - 1);
      memcpy(storebuf, &histbuf[histbuflen - clen], clen);
      if (match_code(storebuf, 7))
        return i;
    }
  } else {
    for (i = 8; i >= 3; i--) {
      clen = 8 - (i - 1);
      memcpy(&storebuf[clen], histbuf, i - 1);
      memset(storebuf, 0, clen);
      if (match_code(storebuf, 7))
        return i;
    }
  }

  /* phase three: scan from the end of the buffer to the current
   * position */
  for (i = histbuflen; i > histbufpos + 8; i--)
    if (match_code(histbuf, i - 1))
      return i;

  /* phase four: scan the remainder to the position, not taking into
   * account unrelated bytes at the other side of the marker */
  for (i = 8; i >= 3; i--) {
    clen = 8 - (i - 1);
    memcpy(&storebuf[clen], &histbuf[histbufpos], i - 1);
    memset(storebuf, 0, clen);
    if (match_code(storebuf, 7))
      return histbufpos + i;
  }

  /* nothing found, return start of the buffer */
  return histbufpos;
}

static uint8_t telnet_read(int id)
{
  client *cl = &clients[id];

  if (cl->connection.available()) {
    uint8_t chr = cl->connection.read();
    if (chr == 0xff) { /* telnet command */
      chr = cl->connection.read();
      switch (chr) {
        case 241: /* NOP */
          /* ignore */
          return 0;
        case 243: /* Break */
          if (cl->state == SCS_SOL) {
            cl->seq = (int)SCSS_BRK;
            cl->connection.write("\r\n[press key to send with "
                                 "Magic SysReq]");
          }
          return 0;
        case 244: /* Interrupt Process */
          /* bring us back to the main menu */
          if (cl->state == SCS_SOL) {
            cl->state = SCS_MENU;
            cl->seq   = 0;
            cl->connection.write("\r\n[interrupt]\r\n");
          }
          return 0;
        case 246: /* Are You There? */
          /* are you there, just answer the question */
          cl->connection.write("Yes!\r\n");
          return 0;
        case 247: /* Erase-Character */
          chr = 127;  /* DEL */
          break;
        case 251: /* WILL */
        case 252: /* WONT */
        case 253: /* DO */
        case 254: /* DONT */
          {
#define mustdo(I) (I == 251 ? 253 : I == 252 ? 253 : I == 253 ? 251 : 251)
#define refuse(I) (I == 251 ? 254 : I == 252 ? 254 : I == 253 ? 252 : 252)
#define opttostr(X) (X == 251 ? "WILL" : X == 252 ? "WON'T" : \
                     X == 253 ? "DO" : "DON'T")
            uint8_t option = cl->connection.read();
            Serial.printf("received %s %u\r\n", opttostr(chr), option);
            switch (option) {
              case 1: /* ECHO */
              case 3: /* SUPPRESS-GO-AHEAD */
              case 45: /* SUPPRESS-LOCAL-ECHO */
                if (option == 1) {
                  if (cl->echo_replied)
                    break;
                  cl->echo_replied = true;
                }
                if (option == 45) {
                  if (cl->localecho_replied)
                    break;
                  cl->localecho_replied = true;
                }
                cl->connection.write(0xff);
                cl->connection.write(mustdo(chr));
                cl->connection.write(option);
                Serial.printf("sent %s %u\r\n", opttostr(mustdo(chr)), option);
                break;
              case 24: /* TERMINAL-TYPE */
              case 31: /* WINDOW-SIZE */
              case 33: /* FLOW-CONTROL */
              case 34: /* LINEMODE */
              default:
                cl->connection.write(0xff);
                cl->connection.write(refuse(chr));
                cl->connection.write(option);
                Serial.printf("sent %s %u\r\n", opttostr(refuse(chr)), option);
                break;
            }
            return 0;
          }
        case 255: /* IAC */
          /* forward to client */
          break;
        default:
          Serial.printf("received unknown command: %u\r\n", chr);
          break;
      }
    }
#ifdef debug
    if (chr < 32 || chr >= 127)
      Serial.printf("received control char %u\r\n", chr);
#endif
    /* map \r<NUL> or \r\n into \n */
    if (chr == '\r')
    {
      if ((chr = cl->connection.read()) == 0)
        chr = '\n';
    }

    return chr;
  }

  return 0;
}

bool telnet_read_string(int id, bool echo, char *buf, int bufsiz, int *buflen)
{
  uint8_t chr;

  while ((chr = telnet_read(id)) != 0) {
    if (chr == '\n') {
      /* hooray, end of string, always echo this to confirm on client side */
      clients_console_write_bytes((char *)&chr, 1, id);
      break;
    } else if (chr == 127) {  /* DEL */
      /* remove from buf, if not at start already */
      if (*buflen > 0)
        (*buflen)--;
      else
        continue;  /* don't echo it back */
    } else {
      /* append char if it fits */
      if (*buflen < bufsiz)
        buf[(*buflen)++] = (char)chr;
    }
    /* echo it back to the client */
    if (echo)
      clients_console_write_bytes((char *)&chr, 1, id);
  }

  return chr != 0;
}

void clients_handle_state(int id)
{
  client *cl = &clients[id];

  switch (cl->state) {
    case SCS_AUTH:
      {
        struct scs_auth *ctx = &cl->statectx.auth;
        client_auth_state seq = (client_auth_state)cl->seq;
        switch (seq) {
          case SCSA_INIT:
            memset(ctx, 0, sizeof(*ctx));
            seq = SCSA_SEND_USER;
            /* request echo to be OFF */
            cl->connection.write(0xff);
            cl->connection.write(251);   /* WILL */
            cl->connection.write(1);     /* ECHO */
            Serial.printf("sent WILL 1 (ECHO)\r\n");

            /* linemode basically is in the way of echo, so disable */
            cl->connection.write(0xff);
            cl->connection.write(254);   /* DONT */
            cl->connection.write(34);    /* LINEMODE */
            Serial.printf("sent DONT 34 (LINEMODE)\r\n");

            /* this is newer, let's request it in any case */
            cl->connection.write(0xff);
            cl->connection.write(253);   /* DO */
            cl->connection.write(45);    /* SUPPRESS-LOCAL-ECHO */
            Serial.printf("sent DO 45 (SUPPRESS-LOCAL-ECHO)\r\n");
            break;
          case SCSA_SEND_USER:
            cl->connection.write((uint8_t *)"username: ", 10);
            seq = SCSA_RECV_USER;
            break;
          case SCSA_SEND_PASS:
            cl->connection.write((uint8_t *)"password: ", 10);
            seq = SCSA_RECV_PASS;
            break;
          case SCSA_RECV_USER:
          case SCSA_RECV_PASS:
            {
              if (seq == SCSA_RECV_USER) {
                if (telnet_read_string(id, true,
                                       ctx->username,
                                       SCSA_BUFMAXLEN,
                                       &ctx->usernamelen))
                  seq = SCSA_SEND_PASS;
              } else if (seq == SCSA_RECV_PASS) {
                if (telnet_read_string(id, false,
                                       ctx->password,
                                       SCSA_BUFMAXLEN,
                                       &ctx->passwordlen))
                  seq = SCSA_CHECK_CRED;
              }
            }
            break;
          case SCSA_CHECK_CRED:
            {
              char user[64];
              char pass[64];
              int userlen = (int)snprintf(user, sizeof(user), "%s",
                                          eeprom_get_var_str(VAR_USERNAME));
              int passlen = (int)snprintf(pass, sizeof(pass), "%s",
                                          eeprom_get_var_str(VAR_PASSWORD));

              if (userlen != ctx->usernamelen ||
                  passlen != ctx->passwordlen ||
                  memcmp(user, ctx->username, userlen) != 0 ||
                  memcmp(pass, ctx->password, passlen) != 0)
              {
                cl->connection.printf("Authentication failure.\r\n\r\n");
                seq = SCSA_SEND_USER;
                ctx->authtries++;
                if (ctx->authtries >= 3)
                  cl->state = SCS_STOP;
              }
              else
              {
                /* login succeeded, advance to the next stage */
                cl->state = SCS_MENU;
                seq = SCSA_INIT;
              }
              /* for security, wipe whatever we got */
              memset(ctx->username, 0, SCSA_BUFMAXLEN);
              memset(ctx->password, 0, SCSA_BUFMAXLEN);
              ctx->usernamelen = 0;
              ctx->passwordlen = 0;
            }
            break;
        }
        cl->seq = (int)seq;
      }
      break;
    case SCS_MENU:
      {
        struct scs_menu *ctx = &cl->statectx.menu;
        client_menu_state seq = (client_menu_state)cl->seq;
        switch (seq) {
          case SCSM_INIT:
            memset(ctx, 0, sizeof(*ctx));
            seq = SCSM_SEND_MAIN;
            cl->connection.printf("SMILO %-12s  %-38s  [%s]\r\n",
                                  GIT_VERSION,
                                  eeprom_get_var_str(VAR_HOSTNAME),
                                  ETH.macAddress().c_str());
            break;
          case SCSM_SEND_MAIN:
            cl->connection.printf("\r\n"
                                  "host board is powered %s\r\n"
                                  "\r\n"
                                  "1. launch [s]erial console\r\n"
                                  "2. view/edit pr[o]perties\r\n"
                                  "3. hit [r]eset button\r\n"
                                  "4. press [p]ower button\r\n"
                                  "5. hold [P]ower button\r\n"
                                  "6. [d]isconnect\r\n"
                                  "7. reboot SMILO\r\n"
                                  "\n"
                                  "Please enter your choice: ",
                                  host_powered_on ?  "on" : "off");
            seq = SCSM_RECV_MAIN;
            break;
          case SCSM_RECV_MAIN:
            {
              uint8_t chr;
              while ((chr = telnet_read(id)) != 0) {
                switch (chr) {
                  case '1':
                  case 's':
                    cl->state = SCS_SOL;
                    seq = SCSM_INIT;
                    cl->connection.printf("serial\r\n\r\n");
                    break;
                  case '2':
                  case 'o':
                    cl->state = SCS_PROP;
                    seq = SCSM_INIT;
                    cl->connection.printf("properties\r\n\n");
                    break;
                  case '3':
                  case 'r':
                    seq = SCSM_SEND_RESET;
                    cl->connection.printf("reset host\r\n\n");
                    break;
                  case '4':
                  case 'p':
                    seq = SCSM_SEND_POWER;
                    cl->connection.printf("power press\r\n\n");
                    break;
                  case '5':
                  case 'P':
                    seq = SCSM_SEND_POWER5S;
                    cl->connection.printf("hold power\r\n\n");
                    break;
                  case '6':
                  case 'q':
                  case 'd':
                    cl->connection.printf("disconnect\r\n\nBye!\r\n");
                    cl->state = SCS_STOP;
                    break;
                  case '7':
                    cl->connection.stop();
                    delay(eeprom_get_var_int(VAR_CYCLEMS));
                    ESP.restart();
                    break;
                  default:
                    if (chr >= ' ' && chr <= '~')
                      cl->connection.printf("\r\n"
                                            "invalid option: '%c'\r\n", chr);
                    seq = SCSM_SEND_MAIN;
                    break;
                }
              }
            }
            break;
          case SCSM_SEND_RESET:
          case SCSM_SEND_POWER:
          case SCSM_SEND_POWER5S:
            {
              char msg[128];
              const char *action;
              size_t msgsiz;
              uint8_t pin;

              action =
                seq == SCSM_SEND_RESET ? "press reset" :
                seq == SCSM_SEND_POWER ? "press power" :
                                         "hold power";

              cl->connection.printf("%s ... ", action);

              msgsiz = snprintf(msg, sizeof(msg),
                                "[request to %s button from %s:%u]\n",
                                action,
                                cl->connection.remoteIP().toString().c_str(),
                                cl->connection.remotePort());
              clients_console_write_bytes(msg, msgsiz, -1);
              clients_history_write_bytes(msg, msgsiz);

              if (seq == SCSM_SEND_RESET)
                pin = RESET_PIN;
              else
                pin = POWER_PIN;

              digitalWrite(pin, HIGH);
              if (seq == SCSM_SEND_POWER5S)
                delay(5000);
              else
                delay(eeprom_get_var_int(VAR_CYCLEMS));
              digitalWrite(pin, LOW);

              cl->connection.printf("done\r\n");

              seq = SCSM_SEND_MAIN;
            }
            break;
        }
        cl->seq = (int)seq;
      }
      break;
    case SCS_PROP:
      {
        struct scs_prop *ctx = &cl->statectx.prop;
        client_prop_state seq = (client_prop_state)cl->seq;
        switch (seq) {
          case SCSP_INIT:
            memset(ctx, 0, sizeof(*ctx));
            seq = SCSP_SEND_PROP;
            break;
          case SCSP_SEND_PROP:
            {
              int i;

              cl->connection.printf("properties:\r\n"
                                   "\n");
              cl->connection.printf(" 1. wipe configuration "
                                    "(reboots SMILO)\r\n");
#define MAX_VAL_SIZE (80 - 3 - 1 - MAX_VARNAME_SIZE - 2 - \
                      2 - 1 - MAX_DEFVAL_SIZE - 1 - 1/* keep one char */)
              for (i = (int)VAR_USERNAME; i <= (int)VAR_LAST; i++) {
                if (eeprom_typemap[i].type == VARTPE_STR) {
                  cl->connection.printf("%2d. %*s  %*s  [%*s]\r\n",
                                       i, -1 * MAX_VARNAME_SIZE,
                                       eeprom_typemap[i].varname,
                                       -1 * MAX_VAL_SIZE,
                                       eeprom_get_var_str((eeprom_var)i),
                                       -1 * MAX_DEFVAL_SIZE,
                                       (char *)eeprom_typemap[i].defval);
                } else if (eeprom_typemap[i].type == VARTPE_INT) {
                  cl->connection.printf("%2d. %*s  %*u  [%*u]\r\n",
                                       i, -1 * MAX_VARNAME_SIZE,
                                       eeprom_typemap[i].varname,
                                       -1 * MAX_VAL_SIZE,
                                       eeprom_get_var_int((eeprom_var)i),
                                       MAX_DEFVAL_SIZE,
                                       (uint32_t)eeprom_typemap[i].defval);
                } else if (eeprom_typemap[i].type == VARTPE_BOL) {
                  cl->connection.printf("%2d. %*s  %*s  [%*s]\r\n",
                                       i, -1 * MAX_VARNAME_SIZE,
                                       eeprom_typemap[i].varname,
                                       -1 * MAX_VAL_SIZE,
                                       eeprom_get_var_bol((eeprom_var)i) ?
                                       "true" : "false",
                                       MAX_DEFVAL_SIZE,
                                       (bool)eeprom_typemap[i].defval ?
                                       "true" : "false");
                }
              }
              cl->connection.printf(" q. return to main menu\r\n"
                                    "\r\n"
                                    "Enter property number: ");
            }
            seq = SCSP_RECV_PROP;
            break;
          case SCSP_RECV_PROP:
            if (!telnet_read_string(id, true,
                                    ctx->buf,
                                    SCSP_BUFMAXLEN - 1,
                                    &ctx->buflen))
              break;
            ctx->buf[ctx->buflen] = '\0';

            /* handle quit, so we can rely on strtol from here */
            if (ctx->buflen == 1 && ctx->buf[0] == 'q') {
              cl->state = SCS_MENU;
              seq = SCSP_INIT;
            }
            else
            {
              long prop = strtol(ctx->buf, NULL, 10);
              if (prop <= 0 || prop > (long)VAR_LAST) {
                cl->connection.printf("invalid property: %s\r\n", ctx->buf);
                seq = SCSP_INIT;
              } else if (prop == 1) {
                cl->connection.println("resetting configuration");
                eeprom_wipe();
                delay(200);
                cl->connection.stop();
                ESP.restart();
              } else {
                ctx->var = eeprom_typemap[prop].var;
                seq = SCSP_RECV_VAL;
                ctx->buflen = 0;
                cl->connection.printf("enter new %s: ",
                                      ctx->var == VAR_USERNAME ? "username" : 
                                      eeprom_typemap[ctx->var].varname);
              }
            }
            break;
          case SCSP_RECV_VAL:
          case SCSP_RECV_PASSCHK:
            if (ctx->var == VAR_PASSWORD) {
              if (!telnet_read_string(id, false,
                                      ctx->buf, SCSA_BUFMAXLEN - 1,
                                      &ctx->buflen))
                break;
              ctx->buf[ctx->buflen] = '\0';

              if (seq == SCSP_RECV_VAL) {
                memcpy(ctx->passcheck, ctx->buf, ctx->buflen);
                ctx->passchecklen = ctx->buflen;
                ctx->buflen = 0;
                seq = SCSP_RECV_PASSCHK;
                cl->connection.printf("enter %s again: ",
                                      eeprom_typemap[VAR_PASSWORD].varname);
                break;
              }

              if (ctx->passchecklen != ctx->buflen ||
                  memcmp(ctx->passcheck, ctx->buf, ctx->buflen) != 0)
              {
                cl->connection.printf("passwords do not match\r\n");
              } else if (ctx->buflen == 0) {
                cl->connection.printf("cannot set password to "
                                      "empty string\r\n");
              } else {
                eeprom_set_var_str(ctx->var, ctx->buf);
              }
              memset(ctx->buf, 0, ctx->buflen);
              memset(ctx->passcheck, 0, ctx->passchecklen);
            } else {
              if (!telnet_read_string(id, true,
                                      ctx->buf,
                                      SCSP_BUFMAXLEN - 1,
                                      &ctx->buflen))
                break;
              ctx->buf[ctx->buflen] = '\0';

              if (eeprom_typemap[ctx->var].type == VARTPE_INT) {
                int val;
                if (ctx->buflen == 0) {
                  val = (int)(eeprom_typemap[ctx->var].defval);
                  cl->connection.printf("setting default value %ld\n", val);
                } else {
                  val = (int)strtol(ctx->buf, NULL, 10);
                }
                eeprom_set_var_int(ctx->var, val);
              } else if (eeprom_typemap[ctx->var].type == VARTPE_STR) {
                if (ctx->buflen == 0) {
                  cl->connection.printf("setting default value %s\n",
                                        eeprom_typemap[ctx->var].defval);
                  eeprom_set_var(ctx->var,
                                 eeprom_typemap[ctx->var].defval);
                }
                else
                {
                  eeprom_set_var_str(ctx->var, ctx->buf);
                }
              } else if (eeprom_typemap[ctx->var].type == VARTPE_BOL) {
                bool val;
                if (ctx->buflen == 0) {
                  val = (bool)(eeprom_typemap[ctx->var].defval);
                  cl->connection.printf("setting default value %s\n",
                                        val ? "true" : "false");
                }
                eeprom_set_var_bol(ctx->var, val);
              }
            }

            if (ctx->var == VAR_USERNAME) {
              /* update password after username */
              ctx->var    = VAR_PASSWORD;
              seq         = SCSP_RECV_VAL;
              ctx->buflen = 0;
              cl->connection.printf("enter new %s: ",
                                    eeprom_typemap[ctx->var].varname);
            } else {
              seq = SCSP_INIT;
            }
            break;
        }
        cl->seq = (int)seq;
      }
      break;
    case SCS_SOL:
      {
        struct scs_sol *ctx = &cl->statectx.sol;
        client_sol_state seq = (client_sol_state)cl->seq;
        switch (seq) {
          case SCSS_INIT:
            {
              size_t pos;

              cl->seq = (int)SCSS_SOL;

              pos = clients_history_find_last_screen();
              if (pos >= histbufpos) {
                clients_console_write_bytes((char *)&histbuf[pos],
                                            histbuflen - pos, id);
                clients_console_write_bytes((char *)histbuf, histbufpos, id);
              } else {
                clients_console_write_bytes((char *)&histbuf[pos],
                                            histbufpos - pos, id);
              }
            }
            break;
          case SCSS_SOL:
            {
              uint8_t chr;

              /* proxy data from clients to uart, byte by byte since this most
               * likely is what the input is (keyboard) */
              while ((chr = telnet_read(id)) != 0)
                Serial1.write(chr);
            }
            break;
          case SCSS_BRK:
            {
              uint8_t chr;

              /* handle BREAK signal from telnet, we turn it into a
               * Magic SysRq, but need to know which one */

              if ((chr = telnet_read(id)) != 0) {
                cl->connection.write(chr);
                cl->connection.write("\r\n");
                /* now send Magic SysRq?
                 * 0x38 Alt
                 * 0x54     + print-screen
                 * ...                     + command char
                 * ...                       release
                 * 0xd4       release
                 * 0xb8 release
                 * now let's do it */
                Serial1.write(0x38);
                Serial1.write(0x54);
                Serial1.write(chr);         /* press */
                Serial1.write(chr + 0x80);  /* release */
                Serial1.write(0xd4);
                Serial1.write(0xb8);
                /* return to console */
                cl->seq = (int)SCSS_SOL;
              }
            }
            break;
        }
      }
      break;
  }
}
/* }}} */

/* {{{ network stack state change handler
 * this sets up the appropriate things such as network listeners and
 * http endpoints when the stack comes up, closes things down, etc.
 * server_ready global is set from here */
void net_handle_event(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(eeprom_get_var_str(VAR_HOSTNAME));

      Serial.printf("MAC address: %s\r\n", ETH.macAddress().c_str());
      Serial.printf("Hostname: %s\r\n", ETH.getHostname());
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.printf("Ethernet link up: %uMbps, %s duplex\r\n",
                    ETH.linkSpeed(), ETH.fullDuplex() ? "full" : "half");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("Received address: %s gw %s dns %s\r\n",
                    ETH.localIP().toString().c_str(),
                    ETH.gatewayIP().toString().c_str(),
                    ETH.dnsIP().toString().c_str());
      Serial.println("Starting telnet server on port 23");
      telnet.begin();
      telnet.setNoDelay(true);
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
      telnet.stopAll();
      http.stop();
      server_ready = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet removed");
      telnet.stopAll();
      telnet.end();
      http.stop();
      http.close();
      server_ready = false;
      break;
    default:
      Serial.printf("unhandled event %u\r\n", event);
      break;
  }
}
/* }}} */

/* {{{ Arduino setup function
 * called once during startup */
void setup() {
  char     eeprom_magic[4];
  bool     initeeprom = false;
  uint32_t baudrate;
  bool     invert;

  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  delay(200);

  /* check magic for EEPROM, else wipe */
  if (EEPROM.readBytes(0, eeprom_magic, 4) != 4 ||
      memcmp(eeprom_magic, EEPROM_MAGIC, 4) != 0)
  {
    eeprom_init();
    initeeprom = true;
  }

  Serial.printf("Setting up ethernet %s\r\n",
                ETH_TYPE == ETH_PHY_LAN8720 ? "LAN8720" : "TLK110");
  WiFi.onEvent(net_handle_event);  /* counter-intuitive: but also for ETH */
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN,
  			    ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

  /* we need MAC address to init, for which we need above ETH.begin */
  if (initeeprom) {
    char  hostnamebuf[32];
    char *p;
    char *q;

    /* set "stock" hostname, that includes unique MAC */
    snprintf(hostnamebuf, sizeof(hostnamebuf), "smilo-%s",
             ETH.macAddress().c_str());
    /* lowercase and remove : to make it valid */
    for (p = hostnamebuf, q = p; *p != '\0'; p++) {
      if ((*p >= '0' && *p <= '9') ||
          (*p >= 'a' && *p <= 'z') ||
          *p == '-' || *p == '_')
        *q++ = *p;
      else if (*p >= 'A' && *p <= 'Z')
        *q++ = tolower(*p);
    }
    *q++ = '\0';
    Serial.printf("Setting default hostname %s\r\n", hostnamebuf);
    eeprom_set_var_str(VAR_HOSTNAME, hostnamebuf);
    /* restart to make the hostname active */
    delay(200);
    ESP.restart();
  }

  host_powered_on = digitalRead(BOARD_PWR_PIN) == HIGH ? true : false;
  baudrate = eeprom_get_var_int(host_powered_on ? VAR_BAUD_ON : VAR_BAUD_OFF);
  invert = eeprom_get_var_bol(VAR_INVERT);
  Serial.printf("Setting up COM1 serial connection: %u baud\r\n", baudrate);
  Serial1.begin(baudrate, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN, invert);

  Serial.println("Setting up power and reset pins for output");
  pinMode(POWER_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  Serial.println("Setting up host board power state pin for input");
  pinMode(BOARD_PWR_PIN, INPUT_PULLDOWN);

  pinMode(ESP_BUT1_PIN, INPUT);
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
  static int laststate = digitalRead(ESP_BUT1_PIN);
  static bool lasthostpoweron = host_powered_on;

  if (digitalRead(ESP_BUT1_PIN) != laststate) {
    Serial.println("Reset button BUT1 pressed, performing EEPROM wipe");
    eeprom_wipe();
    delay(200);
    ESP.restart();
  }

  if (!server_ready)
    return;

  /* track power, and handle state changes */
  host_powered_on = digitalRead(BOARD_PWR_PIN) == HIGH ? true : false;
  if (host_powered_on != lasthostpoweron) {
    char  *msg;
    size_t msgsiz;
    if (host_powered_on) {
      msg    = (char *)"[host powered on]\n";
      msgsiz = 18;
      Serial1.begin(eeprom_get_var_int(VAR_BAUD_ON),
                    SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN, true);
    } else {
      msg    = (char *)"[host powered off]\n";
      msgsiz = 19;
      Serial1.begin(eeprom_get_var_int(VAR_BAUD_OFF),
                    SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN, true);
    }
    clients_console_write_bytes(msg, msgsiz, -1);
    clients_history_write_bytes(msg, msgsiz);
  }
  lasthostpoweron = host_powered_on;

  /* add new connections, if any */
  if (telnet.hasClient()) {
    newclient = telnet.accept();
    if (!newclient) {
      Serial.println("Failed to accept new client, ignoring");
      return;
    }

    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (clients[i].state == SCS_FREE) {
        clients[i].connection = newclient;
        clients[i].connect_at = millis();
        clients[i].state      = SCS_AUTH;
        clients[i].seq        = 0;  /* init */
        break;
      }
    }
    if (i == MAX_SRV_CLIENTS) {
      Serial.println("Cannot accept new connection: too many clients");
      newclient.stop();
      return;
    }

    Serial.printf("New connection from %s:%u\r\n",
                  clients[i].connection.remoteIP().toString().c_str(),
                  clients[i].connection.remotePort());
    clients[i].connection.printf("[successfully connected to %s]\n",
                                 ETH.localIP().toString().c_str());
  }

  /* proxy data from uart to clients and/or buffer, read in blocks
   * output usually is whole lines, this purposely doesn't use a loop so
   * we keep responsive doing other things when somehow the uart is
   * spamming like hell here */
  for (i = 0; (savail = Serial1.available()) > 0 && i < 10; i++) {
    /* unfortunately using clients_history_write_bytes here would
     * require an extra copy, so this bit is an almost copy of that
     * function for the sake of some speed */
    if (histbufpos == sizeof(histbuf))
      histbufpos = 0;
    if (savail > sizeof(histbuf) - histbufpos)
      savail = sizeof(histbuf) - histbufpos;
    savail = Serial1.readBytes(&histbuf[histbufpos], savail);
    clients_console_write_bytes((char *)&histbuf[histbufpos], savail, -1);
    histbufpos += savail;
    if (histbuflen < histbufpos)
      histbuflen = histbufpos;
  }

  connected_clients = 0;
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (clients[i].state != SCS_FREE) {
      if (clients[i].state == SCS_STOP ||
          !clients[i].connection.connected())
      {
        Serial.printf("%s connection from %s:%u after %us\r\n",
                      clients[i].state == SCS_STOP ? "Closed" : "Lost",
                      clients[i].connection.remoteIP().toString().c_str(),
                      clients[i].connection.remotePort(),
                      (millis() - clients[i].connect_at) / 1000);
        clients[i].connection.stop();
        clients[i].state = SCS_FREE;
      } else {
        clients_handle_state(i);

        connected_clients++;
      }
    }
  }

  if (lasttick + 30000 < millis()) {
    if (connected_clients > 0) {
      Serial.printf("Connected clients: %u\r\n", connected_clients);
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (clients[i].state != SCS_FREE) {
          Serial.printf("[%u] %s:%u for %us\r\n",
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
}
/* }}} */

/* vim: set ts=2 sw=2 expandtab foldmethod=marker: */
