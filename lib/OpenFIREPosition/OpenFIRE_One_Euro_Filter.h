#ifdef USE_POS_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_One_Euro_Filter.h
 * @brief OneEuro filter for single point – class declaration
 * @n CPP file for Filtro OneEuro
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 */

// OpenFIRE_One_Euro_Filter.h
// Definition of the Kalman filter class for single point

    #ifndef OPENFIRE_ONE_EURO_FILTER_H
        #define OPENFIRE_ONE_EURO_FILTER_H

        #include <stdint.h>

        #include "OpenFIREConst.h"

class OpenFIRE_One_Euro_Filter {
   public:
    // Constructor: initializes internal state
    OpenFIRE_One_Euro_Filter();

    // Applies the Kalman filter to the point (in/out)
    void One_Euro_Filter(int &outX, int &outY);

   private:
    //================================================================
    // COSTANTI DI TUNING FILTRO ONE EURO
    //================================================================
    static constexpr float OE_FREQ = 200.0f;        // Frequenza campionamento (Hz)
    static constexpr float OE_MIN_CUTOFF = 1.0f;    // Cutoff minimo (Hz)
    static constexpr float OE_BETA = 0.007f;        // Guadagno velocità
    static constexpr float OE_D_CUTOFF = 1.0f;      // Cutoff derivata (Hz)
    static constexpr float OE_DT = 1.0f / OE_FREQ;  // Intervallo campionamento (s)

    //================================================================
    // INTERNAL STATE ONE EURO FILTER (Pos/Vel) – single point
    //================================================================
    // X State
    float oe_x_prev;     // Last raw measurement X
    float oe_x_hat;      // Last filtered value X
    float oe_vel_hat_x;  // Estimated filtered velocity X

    // Y State
    float oe_y_prev;     // Last raw measurement Y
    float oe_y_hat;      // Last filtered value Y
    float oe_vel_hat_y;  // Estimated filtered velocity Y
};

    #endif  // OPENFIRE_ONE_EURO_FILTER_H
#endif      // USE_POS_ONE_EURO_FILTER