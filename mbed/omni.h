#ifndef OMNI_H
#define OMNI_H

namespace OMNI {
    constexpr float F_PI = 3.1514926f;
    constexpr float F_PI_4 = F_PI / 4.0f;
    constexpr float mps = 60.0f;

    template <class T> class OMNI {
        T radial_mm;
        T angular_rad;
        T diameter_of_wheel_mm;
        T zure_rad;
        T gear_ratio;        
    public:
        OMNI(T radial_mm_, T angular_rad_, T diameter_of_wheel_mm_, T zure_rad_, T gear_ratio_)
        :  radial_mm(radial_mm_), angular_rad(angular_rad_), diameter_of_wheel_mm(diameter_of_wheel_mm_), zure_rad(zure_rad_), gear_ratio(gear_ratio_){}

        T return_mmps(T speed_trans_mmps, T direction_trans_rad, T angular_speed_radps) {
            T trans = speed_trans_mmps * (sinf(direction_trans_rad + angular_rad + zure_rad));
            T rot = radial_mm * angular_speed_radps * cosf(zure_rad);
            return -(trans + rot);
        }

        T return_rpm(T speed_trans_mmps, T direction_trans_rad, T angular_speed_radps) {
            return mps * gear_ratio * return_mmps(speed_trans_mmps, direction_trans_rad, angular_speed_radps) / (2 * F_PI * diameter_of_wheel_mm);
        }
    };
}
    


#endif // OMNI_H