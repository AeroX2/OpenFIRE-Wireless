#ifdef USE_POS_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_One_Euro_Filter.cpp
 * @brief OneEuro filter for single point – implementation
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 */

    #include "OpenFIRE_One_Euro_Filter.h"

    #include <cmath>

// Calcola coefficiente alpha da cutoff
static inline float oe_alpha(float cutoff, float dt) {
    const float PI = 3.14159265358979323846f;
    float r = 2.0f * PI * cutoff * dt;
    return r / (r + 1.0f);
}

// Constructor: initializes internal state
OpenFIRE_One_Euro_Filter::OpenFIRE_One_Euro_Filter()
    : oe_x_prev(0.0f), oe_x_hat(0.0f), oe_vel_hat_x(0.0f), oe_y_prev(0.0f), oe_y_hat(0.0f), oe_vel_hat_y(0.0f) {}

// One Euro filter implementation for a single point
void OpenFIRE_One_Euro_Filter::One_Euro_Filter(int &outX, int &outY) {
    // Misura grezza
    float rawX = static_cast<float>(outX);
    float rawY = static_cast<float>(outY);

    // Filtro asse X
    float dx = (rawX - oe_x_prev) / OE_DT;
    float ad = oe_alpha(OE_D_CUTOFF, OE_DT);
    oe_vel_hat_x = ad * dx + (1.0f - ad) * oe_vel_hat_x;
    float cutoffX = OE_MIN_CUTOFF + OE_BETA * std::fabs(oe_vel_hat_x);
    float ax = oe_alpha(cutoffX, OE_DT);
    oe_x_hat = ax * rawX + (1.0f - ax) * oe_x_hat;
    oe_x_prev = rawX;

    // Filtro asse Y
    float dy = (rawY - oe_y_prev) / OE_DT;
    float ady = oe_alpha(OE_D_CUTOFF, OE_DT);
    oe_vel_hat_y = ady * dy + (1.0f - ady) * oe_vel_hat_y;
    float cutoffY = OE_MIN_CUTOFF + OE_BETA * std::fabs(oe_vel_hat_y);
    float ay = oe_alpha(cutoffY, OE_DT);
    oe_y_hat = ay * rawY + (1.0f - ay) * oe_y_hat;
    oe_y_prev = rawY;

    // Output filtrato
    outX = static_cast<int>(oe_x_hat);
    outY = static_cast<int>(oe_y_hat);
}

#endif  // USE_POS_ONE_EURO_FILTER