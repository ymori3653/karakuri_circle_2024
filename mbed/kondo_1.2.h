// 範囲チェック未完了

#ifndef KONDO_H
#define KONDO_H
#include "mbed.h"
#include <cstdint>

constexpr int NUM_SERVO = 1;

// reverse flag
constexpr uint8_t eeprom[] = {0b11000010, 0, 0X5, 0Xa, 0Xf, 0Xe, 0Xf, 0Xf, 0X0, 0X1, 0X0, 0X5, 0X8, 0X0, 0Xf, 0Xa, 0X0, 0XF, 0X2, 0Xc, 0Xe, 0Xc, 0X0, 0Xd, 0Xa, 0Xc, 0X0, 0X0, 0X0, 0Xa, 0X8, 0X0, 0X8, 0X0, 0X0, 0X1, 0X7, 0Xf, 0Xf, 0Xe, 0X9, 0X6, 0Xf, 0X4, 0Xa, 0X0, 0X4, 0X1, 0X9, 0X4, 0X6, 0Xb, 0X9, 0X8, 0Xc, 0X1, 0X9, 0Xd, 0X0, 0XA, 0X0, 0X0, 0X0, 0X0, 0X0, 0X0};

enum class BAUDRATE {
    _115200bps = 115200,
    _625000bps = 625000,
    _1250000bps = 1250000,
};

enum class COMMAND{
    POSITION    = 0b10000000,
    READ        = 0b10100000,
    WRITE       = 0b11000000,
    ID          = 0b11100000,
};

enum class R_COMMAND {
    POSITION    = 0B00000000,
    POSITION2   = 0B10000000,
    READ        = 0B00100000,
    WRITE       = 0B01000000,
    ID          = 0B11100000,
};

enum class SC{
    EEPROM  = 0X00,
    STRC    = 0X01,
    SPD     = 0X02,
    CUR     = 0X03,
    TMP     = 0X04,
    TCH     = 0X05,
};

enum class SC_ID{
    READ  = 0X00,
    WRITE = 0X01,
};

struct Parameters {
    uint8_t EEPROM[100];
    uint8_t STRC;
    uint8_t SPD;
    uint8_t CUR;
    uint8_t TMP;
    uint16_t TCH;

    uint8_t ID;

    Parameters()
      : EEPROM{(0X80)}, 
        STRC(0X80), 
        SPD(0X80), 
        CUR(0X80), 
        TMP(0X80), 
        TCH(0X8000)
    {}
};

struct Data_Received {
    bool is_now_receiving;
    R_COMMAND r_command;
    SC sc;
    uint8_t id;
    uint8_t buf[127];
};


class Kondo {
    RawSerial SERIAL;
    DigitalOut EN_IN;
    Ticker ticker; // EN_IN の管理用。SERIAL.write() がうまくいかないので

    R_COMMAND R_CMD2R_COMMAND(uint8_t R_CMD) { return R_COMMAND(R_CMD & 0B11100000); }
    uint8_t R_CMD2R_ID(uint8_t R_CMD) { return R_CMD & 0B00011111; }

    void turn_off_EN_IN() { // callback の関係上 void(int) 型
        ticker.detach();
        EN_IN.write(0);
        return;
    } 

    
    void set(const uint8_t *buf, uint8_t length) {
        EN_IN.write(1);
        ticker.attach_us(this, &Kondo::turn_off_EN_IN, 1000000 * 11 * length / 115200);
        for (int i = 0; i < length; i++) {
            SERIAL.putc(buf[i]);
        }
    }

    int get_index(uint8_t ID) {
        int index = -1;
        for (int i = 0; i < NUM_SERVO; i++) {
            if (ID == parameters[i].ID) {
                index = i;
            }
        }
        return index;
    }


    void get() {
        //char c;
        static int index = 0;
        while (SERIAL.readable()) {
            data_received.buf[index++] = SERIAL.getc();
        }
        /*static int index;
        static int length_data;

        if(!data_received.is_now_receiving) {
            data_received.is_now_receiving = true;

            uint8_t R_CMD = SERIAL.getc();
            data_received.r_command = R_CMD2R_COMMAND(R_CMD);
            data_received.id = R_CMD2R_ID(R_CMD);

            switch (data_received.r_command) {
            case R_COMMAND::ID: // 1 byte しか返ってこんので、ここで処理
                data_received.is_now_receiving = false;
                parameters[get_index(data_received.id)].ID = R_CMD2R_ID(R_CMD);
                break;

            default:
                break;
            }

            
        }*/


        /*while (SERIAL.readable()) {
            data_received.buf[index++] = SERIAL.getc();
        }

        if (index == length_data - 1) {
            // 解析
        }*/
        return;
    }
    
public:
    uint8_t byte_receive_remained;
    Data_Received data_received;
    Parameters parameters[NUM_SERVO];

    Kondo(PinName tx, PinName rx, PinName en_in, BAUDRATE baudrate)
    : SERIAL(tx, rx, (int)baudrate), EN_IN(en_in, 0), debug(USBTX, USBRX) {
        SERIAL.format(8, SerialBase::Even, 1);
        //SERIAL.attach(Callback<void()>(this, &Kondo::get)); 

        data_received.is_now_receiving = false; 
    }

    
    int set_TCH(uint8_t id, uint16_t position) {
        uint8_t buf[3] = {
            uint8_t((uint8_t)COMMAND::POSITION | id),
            uint8_t(position >> 7),
            uint8_t(position & 0b1111111)
        };
        set(buf, 3);
        return 0;
    }

    int read(uint8_t id, SC sc) {
        uint8_t buf[2] = {
            uint8_t((uint8_t)COMMAND::READ | id),
            uint8_t((uint8_t)sc)
        };
        set(buf, 2);
        return 0;
    }

    int id_command(uint8_t id, SC_ID sc_id) {
        if (sc_id == SC_ID::READ) { id = 0B11111; }
        uint8_t sc = uint8_t(sc_id);

        uint8_t buf[4] = {
            uint8_t((uint8_t)COMMAND::ID | id),
            sc,
            sc,
            sc
        };
        set(buf, 4);
        return 0;      
    }
    
    RawSerial debug;
    int debugcnt;

    void set2() {
        set(eeprom, 66);
    }
};


#endif  // KONDO_H