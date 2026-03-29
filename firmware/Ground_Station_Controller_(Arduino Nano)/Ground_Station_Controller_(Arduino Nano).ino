// =============================================
// BUMBLEBEE-CP — Ground Station Nano Phase 6b
// Builds BB-CP packets, RS encodes, sends via NRF24L01
// Also listens for PONG / TLM_STATUS from drone
//
// WIRING:
//   CE=D7   CSN=D8   SCK=D13   MOSI=D11   MISO=D12
//   NRF24L01 VCC → 3.3V (NOT 5V!)
//
// Serial Monitor: 230400 baud
// Keyboard test keys:
//   1=PING  2=HOVER  3=TAKEOFF  4=FIRE  5=ESTOP  6=LAND
//   7=+1err (inject 1 RS error)  8=+2err (inject 2 RS errors)
// =============================================

#include <avr/pgmspace.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);  // CE=D7, CSN=D8

// =============================================
// GF(256) Tables — PROGMEM
// =============================================
const uint8_t gf_exp[512] PROGMEM = {
1,2,4,8,16,32,64,128,29,58,116,232,205,135,19,38,76,152,45,90,180,117,234,201,143,3,6,12,24,48,96,192,
157,39,78,156,37,74,148,53,106,212,181,119,238,193,159,35,70,140,5,10,20,40,80,160,93,186,105,210,185,111,222,161,
95,190,97,194,153,47,94,188,101,202,137,15,30,60,120,240,253,231,211,187,107,214,177,127,254,225,223,163,91,182,113,226,
217,175,67,134,17,34,68,136,13,26,52,104,208,189,103,206,129,31,62,124,248,237,199,147,59,118,236,197,151,51,102,204,
133,23,46,92,184,109,218,169,79,158,33,66,132,21,42,84,168,77,154,41,82,164,85,170,73,146,57,114,228,213,183,115,
230,209,191,99,198,145,63,126,252,229,215,179,123,246,241,255,227,219,171,75,150,49,98,196,149,55,110,220,165,87,174,65,
130,25,50,100,200,141,7,14,28,56,112,224,221,167,83,166,81,162,89,178,121,242,249,239,195,155,43,86,172,69,138,9,
18,36,72,144,61,122,244,245,247,243,251,235,203,139,11,22,44,88,176,125,250,233,207,131,27,54,108,216,173,71,142,1,
2,4,8,16,32,64,128,29,58,116,232,205,135,19,38,76,152,45,90,180,117,234,201,143,3,6,12,24,48,96,192,157,
39,78,156,37,74,148,53,106,212,181,119,238,193,159,35,70,140,5,10,20,40,80,160,93,186,105,210,185,111,222,161,95,
190,97,194,153,47,94,188,101,202,137,15,30,60,120,240,253,231,211,187,107,214,177,127,254,225,223,163,91,182,113,226,217,
175,67,134,17,34,68,136,13,26,52,104,208,189,103,206,129,31,62,124,248,237,199,147,59,118,236,197,151,51,102,204,133,
23,46,92,184,109,218,169,79,158,33,66,132,21,42,84,168,77,154,41,82,164,85,170,73,146,57,114,228,213,183,115,230,
209,191,99,198,145,63,126,252,229,215,179,123,246,241,255,227,219,171,75,150,49,98,196,149,55,110,220,165,87,174,65,130,
25,50,100,200,141,7,14,28,56,112,224,221,167,83,166,81,162,89,178,121,242,249,239,195,155,43,86,172,69,138,9,18,
36,72,144,61,122,244,245,247,243,251,235,203,139,11,22,44,88,176,125,250,233,207,131,27,54,108,216,173,71,142,1,2
};

const uint8_t gf_log[256] PROGMEM = {
0,0,1,25,2,50,26,198,3,223,51,238,27,104,199,75,4,100,224,14,52,141,239,129,28,193,105,248,200,8,76,113,
5,138,101,47,225,36,15,33,53,147,142,218,240,18,130,69,29,181,194,125,106,39,249,185,201,154,9,120,77,228,114,166,
6,191,139,98,102,221,48,253,226,152,37,179,16,145,34,136,54,208,148,206,143,150,219,189,241,210,19,92,131,56,70,64,
30,66,182,163,195,72,126,110,107,58,40,84,250,133,186,61,202,94,155,159,10,21,121,43,78,212,229,172,115,243,167,87,
7,112,192,247,140,128,99,13,103,74,222,237,49,197,254,24,227,165,153,119,38,184,180,124,17,68,146,217,35,32,137,46,
55,63,209,91,149,188,207,205,144,135,151,178,220,252,190,97,242,86,211,171,20,42,93,158,132,60,57,83,71,109,65,162,
31,45,67,216,183,123,164,118,196,23,73,236,127,12,111,246,108,161,59,82,41,157,85,170,251,96,134,177,187,204,62,90,
203,89,95,176,156,169,160,81,11,245,22,235,122,117,44,215,79,174,213,233,230,231,173,232,116,214,244,234,168,80,88,175
};

// =============================================
// GF Math
// =============================================
uint8_t gf_mul(uint8_t x, uint8_t y) {
    if (!x || !y) return 0;
    return pgm_read_byte(&gf_exp[
        (uint16_t)pgm_read_byte(&gf_log[x]) +
        (uint16_t)pgm_read_byte(&gf_log[y])]);
}
uint8_t gf_div(uint8_t x, uint8_t y) {
    if (!x) return 0;
    return pgm_read_byte(&gf_exp[
        ((uint16_t)pgm_read_byte(&gf_log[x]) + 255 -
         (uint16_t)pgm_read_byte(&gf_log[y])) % 255]);
}
uint8_t gf_pow2(uint8_t p) {
    return pgm_read_byte(&gf_exp[p % 255]);
}

// =============================================
// RS(12,8) Encode
// =============================================
#define RS_DATA    8
#define RS_PARITY  4
#define RS_TOTAL   12

void rs_encode(uint8_t* data, uint8_t* enc) {
    uint8_t g[5] = {1, 15, 54, 120, 64};
    for (int i = 0; i < RS_DATA; i++)   enc[i] = data[i];
    for (int i = 0; i < RS_PARITY; i++) enc[RS_DATA + i] = 0;
    for (int i = 0; i < RS_DATA; i++) {
        uint8_t c = enc[i];
        if (c) {
            enc[i+1] ^= gf_mul(c, g[1]);
            enc[i+2] ^= gf_mul(c, g[2]);
            enc[i+3] ^= gf_mul(c, g[3]);
            enc[i+4] ^= gf_mul(c, g[4]);
        }
    }
    for (int i = 0; i < RS_DATA; i++) enc[i] = data[i];
}

// =============================================
// Protocol Constants
// =============================================
#define BB_START1           0xBB
#define BB_START2           0xEE
#define GS_ID               0x00
#define DRONE_ID            0x01
#define BCAST               0xFF

#define PKT_CMD_FLIGHT      0x01
#define PKT_CMD_FIRE        0x02
#define PKT_EMERGENCY_STOP  0x03
#define PKT_TLM_STATUS      0x04
#define PKT_PING            0x06
#define PKT_PONG            0x07
#define PKT_ACK             0x0A

#define MODE_IDLE    0x00
#define MODE_TAKEOFF 0x01
#define MODE_HOVER   0x02
#define MODE_TRACK   0x03
#define MODE_ALIGN   0x04
#define MODE_FIRE    0x05
#define MODE_LAND    0x06

uint8_t seqNum = 0;

// =============================================
// Build & Send BB-CP Packet over RF
// c1, c2 = byte indices to corrupt (-1 = none)
// =============================================
void sendPacket(uint8_t* data, uint8_t len, uint8_t dst, uint8_t type, int c1, int c2) {
    uint8_t padded[RS_DATA]   = {0};
    uint8_t encoded[RS_TOTAL] = {0};

    for (int i = 0; i < len; i++) padded[i] = data[i];
    rs_encode(padded, encoded);

    // Error injection for test keys 7 & 8
    if (c1 >= 0 && c1 < RS_TOTAL) encoded[c1] ^= 0xFF;
    if (c2 >= 0 && c2 < RS_TOTAL) encoded[c2] ^= 0xFF;

    // Assemble full 32-byte RF packet
    // [BB][EE][SRC][DST][SEQ][TYPE][LEN][12 RS bytes][padding zeros]
    uint8_t pkt[32] = {0};
    pkt[0] = BB_START1;
    pkt[1] = BB_START2;
    pkt[2] = GS_ID;
    pkt[3] = dst;
    pkt[4] = seqNum++;
    pkt[5] = type;
    pkt[6] = len;
    for (int i = 0; i < RS_TOTAL; i++) pkt[7 + i] = encoded[i];

    // TX → Drone → back to RX
    radio.stopListening();
    radio.openWritingPipe((const uint8_t*)"BMBEE");
    bool ok = radio.write(pkt, 32);
    radio.openReadingPipe(1, (const uint8_t*)"GRNDS");
    radio.startListening();

    Serial.print(F("TX TYPE:0x")); Serial.print(type, HEX);
    Serial.print(F(" SEQ:"));      Serial.print(seqNum - 1);
    Serial.print(F(" LEN:"));      Serial.print(len);
    Serial.print(F(" "));
    Serial.println(ok ? F("OK") : F("NO_ACK"));
}

// =============================================
// Receive & Decode PONG / TLM from Drone
// =============================================
void handleIncoming() {
    if (!radio.available()) return;

    uint8_t buf[32] = {0};
    radio.read(buf, 32);

    Serial.print(F("RX PKT: "));
    for (int i = 0; i < 19; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    if (buf[0] != BB_START1 || buf[1] != BB_START2) {
        Serial.println(F("RX: bad start bytes"));
        return;
    }

    uint8_t type = buf[5];

    switch (type) {
        case PKT_PONG: {
            uint16_t uptime = ((uint16_t)buf[7] << 8) | buf[8];
            Serial.print(F("PONG! Drone uptime: "));
            Serial.print(uptime);
            Serial.println(F("s"));
            break;
        }
        case PKT_TLM_STATUS: {
            Serial.print(F("TLM ROLL:"));   Serial.print((int8_t)buf[7]);
            Serial.print(F(" PITCH:"));     Serial.print((int8_t)buf[8]);
            Serial.print(F(" YAW:"));       Serial.print((int8_t)buf[9]);
            Serial.print(F(" BAT:"));       Serial.print(buf[10]);
            Serial.print(F("% MODE:"));     Serial.print(buf[11]);
            uint16_t alt = ((uint16_t)buf[12] << 8) | buf[13];
            Serial.print(F(" ALT:"));       Serial.print(alt);
            Serial.print(F("cm FLAGS:0x")); Serial.println(buf[14], HEX);
            break;
        }
        case PKT_ACK: {
            Serial.print(F("ACK SEQ:"));
            Serial.println(buf[7]);
            break;
        }
        default:
            Serial.print(F("RX Unknown type:0x"));
            Serial.println(type, HEX);
            break;
    }
}

// =============================================
// Setup
// =============================================
void setup() {
    Serial.begin(230400);
    Serial.println(F("BUMBLEBEE-CP Ground Station Nano Phase 6b"));
    Serial.println(F("Initialising NRF24L01..."));

    if (!radio.begin()) {
        Serial.println(F("NRF24L01 FAILED! Check wiring."));
        while (1);
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setPayloadSize(32);
    radio.openReadingPipe(1, (const uint8_t*)"GRNDS");
    radio.startListening();

    Serial.println(F("NRF24L01 OK!"));
    Serial.println(F("-----------------------------------------"));
    Serial.println(F("1=PING 2=HOVER 3=TAKEOFF 4=FIRE 5=ESTOP 6=LAND"));
    Serial.println(F("7=+1err  8=+2err"));
    Serial.println(F("-----------------------------------------"));
}

// =============================================
// Loop
// =============================================
void loop() {
    // Handle replies from drone
    handleIncoming();

    // Handle keyboard commands
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == '1') {
            Serial.println(F("Sending PING..."));
            uint8_t d[] = {};
            sendPacket(d, 0, DRONE_ID, PKT_PING, -1, -1);

        } else if (cmd == '2') {
            Serial.println(F("Sending HOVER..."));
            uint8_t d[] = {0x05, 0xAA, 0, 0, 0, MODE_HOVER, 0, 0};
            sendPacket(d, 8, DRONE_ID, PKT_CMD_FLIGHT, -1, -1);

        } else if (cmd == '3') {
            Serial.println(F("Sending TAKEOFF..."));
            uint8_t d[] = {0x03, 0xE8, 0, 0, 0, MODE_TAKEOFF, 0, 0};
            sendPacket(d, 8, DRONE_ID, PKT_CMD_FLIGHT, -1, -1);

        } else if (cmd == '4') {
            Serial.println(F("Sending FIRE..."));
            uint8_t d[] = {0x00};
            sendPacket(d, 1, DRONE_ID, PKT_CMD_FIRE, -1, -1);

        } else if (cmd == '5') {
            Serial.println(F("Sending EMERGENCY STOP..."));
            uint8_t d[] = {};
            sendPacket(d, 0, BCAST, PKT_EMERGENCY_STOP, -1, -1);

        } else if (cmd == '6') {
            Serial.println(F("Sending LAND..."));
            uint8_t d[] = {0x03, 0xE8, 0, 0, 0, MODE_LAND, 0, 0};
            sendPacket(d, 8, DRONE_ID, PKT_CMD_FLIGHT, -1, -1);

        } else if (cmd == '7') {
            Serial.println(F("Sending HOVER +1 RS error..."));
            uint8_t d[] = {0x05, 0xAA, 0, 0, 0, MODE_HOVER, 0, 0};
            sendPacket(d, 8, DRONE_ID, PKT_CMD_FLIGHT, 2, -1);

        } else if (cmd == '8') {
            Serial.println(F("Sending HOVER +2 RS errors..."));
            uint8_t d[] = {0x05, 0xAA, 0, 0, 0, MODE_HOVER, 0, 0};
            sendPacket(d, 8, DRONE_ID, PKT_CMD_FLIGHT, 1, 5);
        }
    }
}
