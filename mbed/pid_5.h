// © Y. Morinaga 2024

// 060228
// ver. 4 を template にして、広範な型に対応
// 名前空間追加
#ifndef PID_5_H
#define PID_5_H

namespace pid5 {

typedef enum {
    p = 0,
    i = 1,
    d = 2,
} index_pid;

template <class T> class PID {
    T setpoint_previous;
    T input;
    void init_deviation() {
        deviation[p] = deviation[i] = deviation[d] = (T)0;
    }
public:
    T gain[3];
    T deviation[3];
    PID (
        T gain_p_, T gain_i_, T gain_d_,
        T deviation_p_ = (T)0, T deviation_i_ = (T)0, T deviation_d_ = (T)0
    ) : gain{gain_p_, gain_i_, gain_d_}, deviation{(T)0, (T)0, (T)0},
        setpoint_previous((T)0), input((T)0)
    {}

    T get_input() { return input; }

    // absolute_zero: 出力が必ず零になる
    // is_init_deviation: setpoint（内部で記憶）が前回と変わったときに、偏差を零に初期化
    T set_input(bool absolute_zero, bool is_init_deviation, T setpoint, T process_value, T time_defference_s) {
        if (absolute_zero) {
            init_deviation();
            return (T)0;
        }

        if (is_init_deviation) {
            if (setpoint != setpoint_previous) {
                init_deviation();
            }
        }

        setpoint_previous = setpoint;
        
        T deviation_new = setpoint - process_value;

        deviation[d] = (deviation_new - deviation[p]) / time_defference_s;
        deviation[i] += deviation_new * time_defference_s;
        deviation[p] = deviation_new;

        input = (T)0;
        for (int j = 0; j < d; j++) {
            input += deviation[j] * gain[j];
        }

        return input;
    }
};
}


#endif // PID_5_H
// © Y. Morinaga 2024