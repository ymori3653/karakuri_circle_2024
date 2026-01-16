#include <math.h>

constexpr float M_PIf = 3.141592f;
constexpr float M_PI_2f = 3.141592f * .5f;
constexpr float M_PI_4f = 3.141592f * .25f;

#include "mbed.h"

#include "brushless9.h"
#include "esp32_n.h"
#include "omni.h"
#include "pid_5.h"
#include "kondo_1.2.h"
#include <cstdint>
#include <stdint.h>

//--CAN ID-------------------------
constexpr int ID_ARM = 4;
constexpr int ID_ROT = 5;
constexpr int ID_SUSHI_L = 6;
constexpr int ID_SUSHI_R = 7;
//--CAN ID-------------------------


//--ROBOMASTER SPEED---------------
constexpr int RPM_SUSHI = 3500;
constexpr int RPM_ROT = 500;
constexpr int RPM_ARM = 500;
int rpm_arm = 0;
//--ROBOMASTER SPEED---------------

//--ROBOMASTER REVOLUTION RANGE----
constexpr int REV_LAY_SUSHI = -70;
//--ROBOMASTER REVOLUTION RANGE----

//--CHASSIS CONSTANTS--------------
constexpr float CHASSIS_MMPS = 400.0f;//1500.0f;
constexpr float CHASSIS_RADPS = M_PI_4f;//M_PIf;
float chassis_mmps = .0f;
float chassis_rad = .0f;
float chassis_radps = .0f;
//--CHASSIS CONSTANTS--------------

//--FLAGS--------------------------
bool is_lay_sushi = false;
bool is_stand_sushi = false;
bool is_achieved_sushi = true;
//--FLAGS--------------------------



constexpr float TIME_TICKER_US = 3000; 
constexpr float TIME_TICKER_S = (float)TIME_TICKER_US * .000001f;

using namespace pid5;

Timer timer_ticker;
unsigned long long time_ticker;
BrushLess B(PA_11, PA_12);
RawSerial pc(USBTX, USBRX);
AnalogIn switch_rot(PC_4);
AnalogIn switch_2(PA_4);

//--pid-----------------------------
pid5::PID<float> pid_AGE[2]{PID<float>(.0006f, .0002f, .0f), PID<float>(.0006f, .0002f, .0f)};
pid5::PID<float> pid_SAGE[2]{PID<float>(.002f, .0002f, .0f), PID<float>(.002f, .0002f, .0f)};

pid5::PID<float> pids[8]{
    PID<float>(.0010f, .0004f, .0f), 
    PID<float>(.0010f, .0004f, .0f), 
    PID<float>(.0010f, .0004f, .0f), 
    PID<float>(.0010f, .0004f, .0f), 
    PID<float>(.0008f, .0004f, .0f), 
    PID<float>(.0004f, .0002f, .0f), 
    PID<float>(.0004f, .0002f, .0f), 
    PID<float>(.0004f, .0002f, .0f) 
};
//--pid-----------------------------

float CHASSIS_OFFSET = atan2f(204.0f, 150.97f);
OMNI::OMNI<float> omni[4]{
    OMNI::OMNI<float>(209.53f, .0f, 100.0f, .0f, 19.0f),
    OMNI::OMNI<float>(266.2f, -(M_PI_2f + CHASSIS_OFFSET), 100.0f, +CHASSIS_OFFSET, 19.0f),
    OMNI::OMNI<float>(266.2f, +(M_PI_2f + CHASSIS_OFFSET), 100.0f, -CHASSIS_OFFSET, 19.0f),
    OMNI::OMNI<float>(150.97f, M_PIf, 100.0f, .0f, 19.0f),
};

ESP32 kbt(PC_10, PC_11);

Kondo ks(PC_12, PD_2, PA_13, BAUDRATE::_115200bps);
Ticker ticker;

int rpm_rot = 0;
    
char key;
bool is_pushed = false;

uint16_t ID_ks[3] = {10, 11, 12};
uint16_t TCH[3] = {9000, 6000, 7500};


//--prototype----------------------
inline void lay_sushi();
inline void stand_sushi();
inline void no_sushi();

//--prototype----------------------

float debugs = .0f;

void tickerfunc() {
    timer_ticker.reset();
    timer_ticker.start();

    //--BRUSHLESS--------------------------
    //pid5::PID<float>* pidptr = (rpm > 0) ? &pid_AGE[0] : &pid_SAGE[0];
    //pid5::PID<float>* pidptr_ = (rpm < 0) ? &pid_AGE[0] : &pid_SAGE[0];
    //B.SetSpeed(0, pidptr[0].set_input(false, true, +rpm, B.R[0].rpm, TIME_TICKER_S));
    //B.SetSpeed(1, pidptr[1].set_input(false, true, -rpm, B.R[0].rpm, TIME_TICKER_S));


    //--chassis--------------------------------
    
    B.SetSpeed(0, pids[0].set_input(false, true, omni[0].return_rpm(chassis_mmps, chassis_rad, chassis_radps), B.R[0].rpm, TIME_TICKER_S));
    B.SetSpeed(1, pids[1].set_input(false, true, omni[1].return_rpm(chassis_mmps, chassis_rad, chassis_radps), B.R[1].rpm, TIME_TICKER_S));
    B.SetSpeed(2, pids[2].set_input(false, true, omni[2].return_rpm(chassis_mmps, chassis_rad, chassis_radps), B.R[2].rpm, TIME_TICKER_S));
    B.SetSpeed(3, pids[3].set_input(false, true, omni[3].return_rpm(chassis_mmps, chassis_rad, chassis_radps), B.R[3].rpm, TIME_TICKER_S));
    
    //--chassis--------------------------------

    //--rot------------------------------------
    static int cnt_arm = 0;
    static int cnt_rot = 0;

    
    debugs = pids[ID_ARM].set_input(false, true, rpm_arm, B.R[ID_ARM].rpm, TIME_TICKER_S);
    //h && -
    
    B.SetSpeed(ID_ARM, debugs);
    B.SetSpeed(ID_ROT, pids[ID_ROT].set_input(false, true, rpm_rot, B.R[ID_ROT].rpm, TIME_TICKER_S));
    //--rot------------------------------------

    //--sushi----------------------------------
    int rpm_sushi = 0;
    if (is_lay_sushi) {
        is_achieved_sushi = (B.Revolutions[ID_SUSHI_L] < REV_LAY_SUSHI) ? true : false;
        //if (is_achieved_sushi) is_lay_sushi = false;
    }
    else if (is_stand_sushi) {
        is_achieved_sushi = (B.Revolutions[ID_SUSHI_L] > 0) ? true : false;
        //if (is_achieved_sushi) is_stand_sushi = false;
    }
    if (is_lay_sushi && !is_achieved_sushi) rpm_sushi = -RPM_SUSHI;
    else if (is_stand_sushi && !is_achieved_sushi) rpm_sushi = +RPM_SUSHI;
    else rpm_sushi = 0;
    B.SetSpeed(ID_SUSHI_L, pids[ID_SUSHI_L].set_input(false, true, rpm_sushi, B.R[ID_SUSHI_L].rpm, TIME_TICKER_S));
    B.SetSpeed(ID_SUSHI_R, pids[ID_SUSHI_R].set_input(false, true, rpm_sushi, B.R[ID_SUSHI_R].rpm, TIME_TICKER_S));
    //--sushi----------------------------------
    
    /*switch (key) {
    case 'a':
        rpm_sushi = +RPM_SUSHI;
        break;
    case 'b':
        rpm_sushi = -RPM_SUSHI;
        break;
    default:
        rpm_sushi = 0;
    }*/
    B.Write();
    //--BRUSHLESS--------------------------

    //pidptr_[0].set_input(false, true, 0, B.R[0].rpm, TIME_TICKER_S);
    //pidptr_[1].set_input(false, true, 0, B.R[0].rpm, TIME_TICKER_S);

    //if (is_pushed) {
    //    is_pushed = false;
    //    ks.set_TCH(ID_ks[2], TCH[2]);
    //}
    

    timer_ticker.stop();
}





// main() runs in its own thread in the OS
int main()
{
    kbt.init(153600);
    B.Init();
    ticker.attach_us(tickerfunc, TIME_TICKER_US);
    while (true) {
        if(pc.readable()) {
            is_pushed = true;
            key = pc.getc();



            switch (key) {
            case 'a':
                lay_sushi();
                break;
            case 'b':
                stand_sushi();
                break;
            case 'c':
                no_sushi();
                break;

            case 'd':
                rpm_rot = +200;
                break;
            case 'e':
                rpm_rot = -200;
                no_sushi();
                break;
            case 'f':
                rpm_rot = 0;
                break;

            case 'g':
                rpm_arm = +RPM_ARM;
                break;
            case 'h':
                rpm_arm = -RPM_ARM;
                break;
            case 'i':
                rpm_arm = 0;
                break;

            
            case 'j':
                chassis_mmps = .0f;
                chassis_rad = .0f;
                chassis_radps = M_PI_4f * .5f;
                break;

            case 'k':
                chassis_mmps = .0f;
                chassis_rad = M_PIf;
                chassis_radps = -M_PI_4f * .5f;
                break;

            case 'l':
                chassis_mmps = .0f;
                chassis_radps = .0f;
                break;


            case 'p':
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = .0f;
                chassis_radps = .0f;
                break;

            case 'q':
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PI_2f;
                chassis_radps = .0f;
                break;

            case 'r':
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PIf;
                chassis_radps = .0f;
                break;

            case 's':
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PI_2f * 3.0f;
                chassis_radps = .0f;
                break;

            case 't':
                chassis_mmps = .0f;
                chassis_radps = +CHASSIS_RADPS;
                
                break;

            case 'u':
                chassis_mmps = .0f;
                chassis_radps = -CHASSIS_RADPS;
                break;

            case 'v':
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PI_4f * 1.5f;
                chassis_radps = .0f;
                break;



            case 'z':
                no_sushi();
                rpm_rot = 0;
                rpm_arm = 0;
                chassis_mmps = .0f;
                chassis_radps = .0f;
                break;
            default:
                //rpm = 0;
                //no_sushi();
                break;
            }

            /*switch (key) {
            case 'c':
                TCH[2] += 100;
                break;
            case 'd':
                TCH[2] -= 100;
                break;
            case 'e':
                TCH[1] += 100;
                break;
            case 'f':
                TCH[1] -= 100;
                break;
            }*/
            
        }
        else {
            if (kbt.Button[Up]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = .0f;
                chassis_radps = .0f;
            }
            else if (kbt.Button[Left]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PI_2f;
                chassis_radps = .0f;
            }
            else if (kbt.Button[Down]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PIf;
                chassis_radps = .0f;
            }
            else if (kbt.Button[Right]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PI_2f * 3.0f;
                chassis_radps = .0f;
            }
            /*else if (kbt.Button[Circle]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = -M_PIf / 6.0f;
                chassis_radps = .0f;
            }
            else if (kbt.Button[Cross]) {
                chassis_mmps = CHASSIS_MMPS;
                chassis_rad = M_PIf + -M_PIf / 6.0f;
                chassis_radps = .0f;
            }*/
            
            else if (kbt.Button[L1]) {
                chassis_mmps = .0f;
                chassis_radps = +CHASSIS_RADPS;
            }
            else if (kbt.Button[R1]) {
                chassis_mmps = .0f;
                chassis_radps = -CHASSIS_RADPS;
            }
            //else if (kbt.Button[Circle]) {
            //    chassis_mmps = CHASSIS_MMPS;
            //    chassis_rad = M_PI_4f * 1.5f;
            //    chassis_radps = .0f;
            //}
            else {
                chassis_mmps = .0f;
                chassis_rad = M_PI_4f * 1.5f;
                chassis_radps = .0f;
            }

            // 動画撮影時は以下のコメントアウトを使用。
            if (kbt.Button[Cross]) {
                rpm_arm = +RPM_ARM;
            }
            else if (kbt.Button[Triangle]) {
                rpm_arm = -RPM_ARM;
            }
            else {
                rpm_arm = 0;
            }

            if (kbt.Button[Circle]) {
                rpm_rot = -RPM_ROT;
            }
            else if (kbt.Button[Square]) {
                rpm_rot = +RPM_ROT;
            }
            else {
                rpm_rot = 0;
            }

            if (kbt.Button[R2]) {
                if (is_lay_sushi) stand_sushi();
                else lay_sushi();
            }

            if (kbt.Button[Options]) {
                TCH[1] += 300;
                TCH[2] += 300;
            }
            else if (kbt.Button[Share]) {
                TCH[1] -= 300;
                TCH[2] -= 300;
            }

            if (kbt.Button[L2]) {
                TCH[0] = (TCH[0] == 9000) ? 7000 : 9000;
            }

        }
        //--kondo---------------------------------------------
        static int cnt = 0;
        cnt++;
        ks.set_TCH(ID_ks[cnt % 3], TCH[cnt % 3]);
        //--kondo---------------------------------------------

        //--switch--------------------------------------------
        static int cnt_arm = 0;
        if(switch_2.read_u16() < 60000) cnt_arm++;
        else cnt_arm = 0;
        if (cnt_arm > 3) B.Revolutions[ID_ARM] = 0;
        //--switch--------------------------------------------

        //ks.set_TCH(ID_ks[1], TCH[1]);
        //pc.printf("time: %lld, %d, rpm: %d, TC: %d, %d, %d, %d\n", timer_ticker.read_high_resolution_us(), rpm, B.R[0].rpm, B.R[0].TC, TCH[0], TCH[1], TCH[2]);
        //pc.printf("time: %lld, %+4d, rpm: %+5d, TC: %+5d, rev: %+3d, switch_rot: %5d, DEBUG:%d\n", timer_ticker.read_high_resolution_us(), rpm, B.R[ID_ROT].rpm, B.R[ID_ROT].TC, B.Revolutions[ID_ROT], switch_rot.read_u16(), B.R[ID_SUSHI_L].TC);
        pc.printf("time: %lld, %+4d -> %+4d, TC: %+5d, rev: %+3d, switch_rot: %5d, %5d, DEBUG:%d, debugs: %f, %d\n", timer_ticker.read_high_resolution_us(), rpm_arm ,B.R[ID_ARM].rpm, B.R[ID_ARM].TC, B.Revolutions[ID_ARM], switch_rot.read_u16(), switch_2.read_u16(), B.R[ID_SUSHI_L].TC, debugs, TCH[0]);
        wait_us(10000);
    }
    
}




//8900
inline void lay_sushi() {
    is_lay_sushi = true;
    is_stand_sushi = false;
}
inline void stand_sushi() {
    is_stand_sushi = true;
    is_lay_sushi = false;
}
inline void no_sushi() {
    is_stand_sushi = false;
    is_lay_sushi = false;
}
