#pragma once
#include <cmath>
#include "Adafruit_BNO08x.h"

struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

Vec3 quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    Vec3 euler;
    euler.x = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));        // roll
    euler.y = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));       // yaw
    euler.z = -1 * atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));  // pitch
    return euler;
}

Vec3 quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);

}

Vec3 quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}