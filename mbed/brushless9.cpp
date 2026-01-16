// from brushless8
// brushless9: to memorize absolute coordination
// made by Morinaga

#include "brushless9.h"

BrushLess::BrushLess(PinName CAN_RX, PinName CAN_TX) : can(CAN_RX, CAN_TX, 1000000) /*srart change by Morinaga*/, Revolutions{0, 0, 0, 0, 0, 0, 0, 0}/*end change by Morinaga*/{
}

void BrushLess::CanIntr() {
    CANMessage msg;
    can.read(msg);
    char data[8] = {msg.data[1],msg.data[0],msg.data[3],msg.data[2],msg.data[5],msg.data[4],msg.data[6], msg.data[7]};
    for (int i = 0; i < 8; i++) {
        if (msg.id == CAN_RECEIVE_ID[i]) {
            // srart change by Morinaga
            short degree_prev = R[i].degree;
            // end   change by Morinaga

            memcpy((ReceiveData*)&(R[i]), data, 8);

            // srart change by Morinaga
            short difference = R[i].degree - degree_prev;
            if      (difference < -DEGREE_RECOGNITION_REVOLUTION) { Revolutions[i]++; }
            else if (difference > +DEGREE_RECOGNITION_REVOLUTION) { Revolutions[i]--; }
            // end   change by Morinaga
            
        }
    }
    // srart change by Morinaga
    //cnt++;
    // srart change by Morinaga
}

void BrushLess::Init() {
    can.attach(callback(this, &BrushLess::CanIntr));
}

void BrushLess::SetSpeed(int id, float speed) {
    float sp = speed;
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    S.speed[id] = 10000 * sp;
}

bool BrushLess::Write() {
    for (int i = 0; i < 2; i++) {
        char b[8];
        memcpy((char*)b, (&S)->speed + (4 * i), 8);
        char send[8] = {b[1], b[0], b[3],b[2],b[5],b[4],b[7],b[6]};
        if (can.write(CANMessage(CAN_SEND_ID[i], send, 8)) == 0) {
        return false;
        }
    }
    return true;
}