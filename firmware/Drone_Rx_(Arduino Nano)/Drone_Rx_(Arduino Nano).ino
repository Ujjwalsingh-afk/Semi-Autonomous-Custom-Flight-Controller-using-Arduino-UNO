// =============================================
// BUMBLEBEE-CP — Drone Nano Phase 6 (FINAL)
// NRF24L01 receive + RS decode + forward to UNO
// CE=D7 CSN=D8 SCK=D13 MOSI=D11 MISO=D12
// SoftwareSerial: D10(TX)->UNO D11  D4(RX)<-UNO D12
// Baud: 230400
// =============================================

#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);
SoftwareSerial unoSerial(4, 10);  // RX=D4, TX=D10 — no conflict with MOSI D11

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

#define RS_DATA    8
#define RS_PARITY  4
#define RS_TOTAL   12

void rs_encode(uint8_t* data, uint8_t* enc) {
    uint8_t g[5]={1,15,54,120,64};
    for(int i=0;i<RS_DATA;i++) enc[i]=data[i];
    for(int i=0;i<RS_PARITY;i++) enc[RS_DATA+i]=0;
    for(int i=0;i<RS_DATA;i++){
        uint8_t c=enc[i];
        if(c){
            enc[i+1]^=gf_mul(c,g[1]);
            enc[i+2]^=gf_mul(c,g[2]);
            enc[i+3]^=gf_mul(c,g[3]);
            enc[i+4]^=gf_mul(c,g[4]);
        }
    }
    for(int i=0;i<RS_DATA;i++) enc[i]=data[i];
}

int8_t rs_decode(uint8_t* msg, uint8_t* out) {
    uint8_t S[4]={0,0,0,0};
    for(int i=0;i<4;i++)
        for(int j=0;j<RS_TOTAL;j++)
            S[i]=msg[j]^gf_mul(S[i],gf_pow2(i));
    if(!S[0]&&!S[1]&&!S[2]&&!S[3]){
        for(int i=0;i<RS_DATA;i++) out[i]=msg[i];
        return 0;
    }
    if(S[0]){
        uint8_t k=pgm_read_byte(&gf_log[gf_div(S[1],S[0])]);
        uint8_t idx=RS_TOTAL-1-k;
        if(idx<RS_TOTAL){
            uint8_t test[RS_TOTAL];
            for(int i=0;i<RS_TOTAL;i++) test[i]=msg[i];
            test[idx]^=S[0];
            uint8_t S2[4]={0,0,0,0};
            for(int i=0;i<4;i++)
                for(int j=0;j<RS_TOTAL;j++)
                    S2[i]=test[j]^gf_mul(S2[i],gf_pow2(i));
            if(!S2[0]&&!S2[1]&&!S2[2]&&!S2[3]){
                for(int i=0;i<RS_DATA;i++) out[i]=test[i];
                return 1;
            }
        }
    }
    uint8_t det=gf_mul(S[0],S[2])^gf_mul(S[1],S[1]);
    if(!det) return -1;
    uint8_t sigma2=gf_div(gf_mul(S[2],S[2])^gf_mul(S[1],S[3]),det);
    uint8_t sigma1=gf_div(gf_mul(S[0],S[3])^gf_mul(S[1],S[2]),det);
    uint8_t roots[2]; uint8_t nr=0;
    for(int i=0;i<RS_TOTAL&&nr<2;i++){
        uint8_t xi=gf_pow2(i);
        if(!(gf_mul(xi,xi)^gf_mul(sigma1,xi)^sigma2)) roots[nr++]=i;
    }
    if(nr!=2) return -1;
    uint8_t X1=gf_pow2(roots[0]),X2=gf_pow2(roots[1]);
    uint8_t dn=X1^X2; if(!dn) return -1;
    uint8_t e1=gf_div(S[1]^gf_mul(S[0],X2),dn);
    uint8_t e2=S[0]^e1;
    uint8_t p1=RS_TOTAL-1-roots[0], p2=RS_TOTAL-1-roots[1];
    uint8_t test[RS_TOTAL];
    for(int i=0;i<RS_TOTAL;i++) test[i]=msg[i];
    if(p1<RS_TOTAL) test[p1]^=e1;
    if(p2<RS_TOTAL) test[p2]^=e2;
    uint8_t S2[4]={0,0,0,0};
    for(int i=0;i<4;i++)
        for(int j=0;j<RS_TOTAL;j++)
            S2[i]=test[j]^gf_mul(S2[i],gf_pow2(i));
    if(!S2[0]&&!S2[1]&&!S2[2]&&!S2[3]){
        for(int i=0;i<RS_DATA;i++) out[i]=test[i];
        return 2;
    }
    return -1;
}

#define BB_START1           0xBB
#define BB_START2           0xEE
#define GS_ID               0x00
#define MY_ID               0x01
#define BCAST               0xFF
#define PKT_CMD_FLIGHT      0x01
#define PKT_CMD_FIRE        0x02
#define PKT_EMERGENCY_STOP  0x03
#define PKT_TLM_STATUS      0x04
#define PKT_PING            0x06
#define PKT_PONG            0x07
#define MODE_IDLE    0x00
#define MODE_TAKEOFF 0x01
#define MODE_HOVER   0x02
#define MODE_TRACK   0x03
#define MODE_ALIGN   0x04
#define MODE_FIRE    0x05
#define MODE_LAND    0x06

void forwardToUNO(uint8_t type, uint8_t* data, uint8_t len) {
    unoSerial.write(0xBB);
    unoSerial.write(0xEE);
    unoSerial.write(type);
    unoSerial.write(len);
    uint8_t crc=0xBB^0xEE^type^len;
    for(int i=0;i<len;i++){unoSerial.write(data[i]);crc^=data[i];}
    unoSerial.write(crc);
    Serial.print(F("->UNO TYPE:0x")); Serial.print(type,HEX);
    Serial.print(F(" LEN:")); Serial.println(len);
}

void sendPong() {
    uint8_t padded[RS_DATA]={0},encoded[RS_TOTAL]={0};
    uint16_t uptime=(uint16_t)(millis()/1000);
    padded[0]=uptime>>8; padded[1]=uptime&0xFF;
    rs_encode(padded,encoded);
    uint8_t pkt[32]={0};
    pkt[0]=0xBB;pkt[1]=0xEE;pkt[2]=MY_ID;
    pkt[3]=GS_ID;pkt[4]=0x01;pkt[5]=PKT_PONG;pkt[6]=2;
    for(int i=0;i<RS_TOTAL;i++) pkt[7+i]=encoded[i];
    radio.stopListening();
    radio.openWritingPipe((const uint8_t*)"GRNDS");
    radio.write(pkt,32);
    radio.startListening();
    Serial.println(F("PONG sent"));
}

void processPayload(uint8_t type, uint8_t* data, uint8_t len) {
    switch(type){
        case PKT_CMD_FLIGHT:{
            uint16_t thr=((uint16_t)data[0]<<8)|data[1];
            Serial.print(F("FLT T:")); Serial.print(thr);
            Serial.print(F(" P:")); Serial.print((int8_t)data[2]);
            Serial.print(F(" R:")); Serial.print((int8_t)data[3]);
            Serial.print(F(" Y:")); Serial.print((int8_t)data[4]);
            Serial.print(F(" M:")); Serial.println(data[5]);
            forwardToUNO(type,data,len); break;
        }
        case PKT_CMD_FIRE:
            Serial.println(F("FIRE!"));
            forwardToUNO(type,data,len); break;
        case PKT_EMERGENCY_STOP:
            Serial.println(F("ESTOP!"));
            forwardToUNO(type,data,len); break;
        case PKT_PING:
            Serial.println(F("PING → sending PONG"));
            sendPong(); break;
        default:
            Serial.print(F("Unknown:0x")); Serial.println(type,HEX); break;
    }
}

enum State{WAIT_BB,WAIT_EE,READ_SRC,READ_DST,READ_SEQ,READ_TYPE,READ_LEN,READ_PAYLOAD,READ_RS_PARITY};
State parserState=WAIT_BB;
uint8_t pkt_src,pkt_dst,pkt_seq,pkt_type,pkt_len;
uint8_t rsBlock[RS_TOTAL],decoded[RS_DATA];
uint8_t payloadIndex=0,parityIndex=0;

void rsComplete(){
    for(int i=pkt_len;i<RS_DATA;i++) rsBlock[i]=0;
    int8_t result=rs_decode(rsBlock,decoded);
    if(result>=0){
        if(result>0){
            Serial.print(F("RS fixed "));
            Serial.print(result);
            Serial.println(F(" error(s)"));
        }
        if(pkt_dst==MY_ID||pkt_dst==BCAST)
            processPayload(pkt_type,decoded,pkt_len);
    } else {
        Serial.println(F("RS FAIL-dropped"));
    }
}

void parseByte(uint8_t b){
    switch(parserState){
        case WAIT_BB:   if(b==BB_START1) parserState=WAIT_EE; break;
        case WAIT_EE:   parserState=(b==BB_START2)?READ_SRC:WAIT_BB; break;
        case READ_SRC:  pkt_src=b;  parserState=READ_DST;  break;
        case READ_DST:  pkt_dst=b;  parserState=READ_SEQ;  break;
        case READ_SEQ:  pkt_seq=b;  parserState=READ_TYPE; break;
        case READ_TYPE: pkt_type=b; parserState=READ_LEN;  break;
        case READ_LEN:  pkt_len=b; payloadIndex=0; parityIndex=0; parserState=READ_PAYLOAD; break;
        case READ_PAYLOAD:
            rsBlock[payloadIndex++]=b;
            if(payloadIndex>=pkt_len) parserState=READ_RS_PARITY; break;
        case READ_RS_PARITY:
            rsBlock[RS_DATA+parityIndex++]=b;
            if(parityIndex>=RS_PARITY){rsComplete();parserState=WAIT_BB;} break;
    }
}

enum UNOState{U_WAIT_BB,U_WAIT_EE,U_TYPE,U_LEN,U_DATA,U_CRC};
UNOState unoState=U_WAIT_BB;
uint8_t unoType,unoLen,unoData[8],unoIdx,calcUNOCRC;

void parseUNOByte(uint8_t b){
    switch(unoState){
        case U_WAIT_BB: if(b==0xBB) unoState=U_WAIT_EE; break;
        case U_WAIT_EE: unoState=(b==0xEE)?U_TYPE:U_WAIT_BB; break;
        case U_TYPE: unoType=b; calcUNOCRC=0xBB^0xEE^b; unoState=U_LEN; break;
        case U_LEN: unoLen=b; calcUNOCRC^=b; unoIdx=0;
            unoState=(unoLen>0)?U_DATA:U_CRC; break;
        case U_DATA: unoData[unoIdx++]=b; calcUNOCRC^=b;
            if(unoIdx>=unoLen) unoState=U_CRC; break;
        case U_CRC:
            if(b==calcUNOCRC){
                Serial.print(F("<-UNO TLM TYPE:0x"));
                Serial.println(unoType,HEX);
            } else {
                Serial.println(F("<-UNO CRC ERR"));
            }
            unoState=U_WAIT_BB; break;
    }
}

void setup(){
    Serial.begin(230400);
    unoSerial.begin(38400);
    Serial.println(F("BUMBLEBEE-CP Drone Nano Phase 6 (FINAL)"));
    Serial.println(F("Initialising NRF24L01..."));
    if(!radio.begin()){
        Serial.println(F("NRF24L01 FAILED! Check wiring."));
        while(1);
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setPayloadSize(32);
    radio.openReadingPipe(1,(const uint8_t*)"BMBEE");
    radio.startListening();
    Serial.println(F("NRF24L01 OK!"));
    Serial.println(F("Listening for packets..."));
    Serial.println(F("─────────────────────────"));
}

void loop(){
    if(radio.available()){
        uint8_t buf[32];
        radio.read(buf,32);
        Serial.print(F("RF PKT: "));
        for(int i=0;i<19;i++){
            if(buf[i]<0x10) Serial.print('0');
            Serial.print(buf[i],HEX);
            Serial.print(' ');
        }
        Serial.println();
        for(int i=0;i<32;i++) parseByte(buf[i]);
    }
    if(unoSerial.available()) parseUNOByte(unoSerial.read());
}
