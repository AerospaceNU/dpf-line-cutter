#include "altitude_kalman.h"
#include <Arduino.h>

AltitudeKalman::AltitudeKalman() {}

#define DEBUG 1

void AltitudeKalman::Predict(const double az, const double dt) {
    // x_k+1 = A x_k + B u_k
    // x_k+1 = [pos] = [old pos + vel * m_dt] + [1/2 acceleration m_dt^2]
    //         [vel]   [old vel           ]   [acceleration * m_dt    ]
    #ifdef DEBUG
    Serial.print("Az "); Serial.println(az);
    #endif
    xHat.estimatedAltitude += xHat.estimatedVelocity * dt + 1/2 * az * dt * dt;
    xHat.estimatedVelocity += az * dt;
}

void AltitudeKalman::Correct(const double baroAltitude, const double kalman_gain[2]) {
    // xhat = xhat + K(y-Cx-Du)
    // x_corrected = [old pos] + [k1] * [ [barometric altitude] - [1, 0] * [estimated pos] ]
    //               [old vel]   [k2]   [                                  [estimated vel] ]

    // The delta between y (where we think we are) and xhat (where we actually are) with
    // all the zero terms omitted is actually pretty clean!
    double deltaAltitude = baroAltitude - xHat.estimatedAltitude;
    #ifdef DEBUG
    Serial.print("Error "); Serial.println(deltaAltitude);
    Serial.print("estimatedAltitude "); Serial.println(xHat.estimatedAltitude);
    Serial.print("estimatedVelocity "); Serial.println(xHat.estimatedVelocity);
    #endif
    
    xHat.estimatedAltitude += kalman_gain[0] * deltaAltitude;
    xHat.estimatedVelocity += kalman_gain[1] * deltaAltitude;
}

const AltitudeKalmanOutput_t AltitudeKalman::GetXhat() const {
    return xHat;
}
