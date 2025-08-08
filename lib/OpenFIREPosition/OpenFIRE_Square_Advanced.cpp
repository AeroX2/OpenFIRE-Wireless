#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_Square_Advanced.cpp
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 *
 * I thank you for producing the first original code:
 *
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 */

    #include "OpenFIRE_Square_Advanced.h"

    #include <Arduino.h>

    #include <cfloat>

    // Definition of indices for clarity
    #define A 0
    #define B 1
    #define C 2
    #define D 3

void OpenFIRE_Square::begin(const int* px, const int* py, unsigned int seen) {
    // --- PHASE 0: Initialization and Visibility Check ---

    seenFlags = seen;

    // The system requires seeing all 4 sensors at least once to initialize.
    if (seenFlags == 0x0F) {  // 0x0F in binary is 1111, all sensors seen.
        start = 0xFF;
    } else if (!start) {
        return;  // Exit if we haven't been initialized yet.
    }

    // --- FASE 1: Estrazione e Pre-elaborazione dei Punti Visti ---

    uint8_t num_points_seen = 0;
    int positionXX[4];
    int positionYY[4];

    // Extracts visible points, puts them in work arrays and applies transformation.
    for (uint8_t i = 0; i < 4; ++i) {
        if ((seenFlags >> i) & 0x01) {
            positionXX[num_points_seen] = (CamMaxX - px[i]) << CamToMouseShift;
            positionYY[num_points_seen] = py[i] << CamToMouseShift;
            num_points_seen++;
        }
    }

    // Proceed only if we have enough data to work with (at least 2 points).
    //////if (num_points_seen > 1) {
    if (num_points_seen >= 1) {
        // just_reacquired_tracking = true; // We have just reacquired the points

        // Declaration of all local variables used in various logical blocks.
        // This resolves "scope" variable errors.
        int32_t dx, dy;
        uint8_t a, b, c, d;
        uint8_t P1, P2;
    #ifdef COMMENTO
        if (num_points_seen == 1) {
            // ============================================================================
            // 1 SENSOR MANAGEMENT (Version with Final Correction)
            // ============================================================================

            const int x1 = positionXX[0];
            const int y1 = positionYY[0];
            uint8_t identified_idx = 0;
            bool identified_successfully = false;

            // --- PHASE 1: IDENTIFICATION ---
            const float shortest_side = fminf(width, height);
            const float dynamic_threshold_sq = (shortest_side / 2.0f) * (shortest_side / 2.0f);

            if (prev_num_points_seen >= 2)  // Transition from stable
            {
                float min_dist_sq = -1.0f;
                uint8_t best_guess_idx = 0;
                for (uint8_t i = 0; i < 4; i++) {
                    if (prev_point_seen_mask & (1 << (3 - i))) {
                        const float dx = (float)x1 - FinalX[i], dy = (float)y1 - FinalY[i];
                        const float dist_sq = dx * dx + dy * dy;
                        if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_guess_idx = i;
                        }
                    }
                }
                if (min_dist_sq >= 0 && min_dist_sq <= dynamic_threshold_sq) {
                    identified_idx = best_guess_idx;
                    identified_successfully = true;
                }
            } else if (prev_num_points_seen == 1)  // Continued tracking
            {
                const uint8_t candidate_idx = last_identified_idx;
                const float dx = (float)x1 - FinalX[candidate_idx], dy = (float)y1 - FinalY[candidate_idx];
                const float dist_sq = dx * dx + dy * dy;
                if (dist_sq <= dynamic_threshold_sq) {
                    identified_idx = candidate_idx;
                    identified_successfully = true;
                }
            }

            // --- PHASE 2: RECONSTRUCTION AND DATA PREPARATION ---
            if (identified_successfully) {
                // 1. Calculate the reconstructed rectangle in temporary buffers
                const float delta_x = (float)x1 - (float)FinalX[identified_idx];
                const float delta_y = (float)y1 - (float)FinalY[identified_idx];

                float Vx[4], Vy[4];
                for (int i = 0; i < 4; i++) {
                    Vx[i] = (float)FinalX[i] + delta_x;
                    Vy[i] = (float)FinalY[i] + delta_y;
                }

                // 2. CRUCIAL UPDATE (THE CORRECTION)
                // We populate positionXX and positionYY with the reconstructed data.
                // In this way, your subsequent common code will find the correct data
                // and will be able to reorganize FinalX/Y as expected.
                for (int i = 0; i < 4; i++) {
                    positionXX[i] = roundf(Vx[i]);
                    positionYY[i] = roundf(Vy[i]);
                }

                // 3. Update other state variables
                is_tracking_stable = true;
                last_identified_idx = identified_idx;
                // The mask now indicates that ALL points are "known" (even if reconstructed)
                current_point_seen_mask = 0b00001111;
            } else {
                is_tracking_stable = false;
                current_point_seen_mask = 0;
            }
        }
    #endif

        // This block goes after the logic for 4 and 2 sensors
        if (num_points_seen == 1) {
            // --- CONDITION FOR ENTRY (CORRECT) ---
            // Proceed if the previous frame was not completely dark.
            if (prev_num_points_seen >= 1) {
                /*
                 * OBJECTIVE:
                 * Find the index of the closest vertex, using the technique
                 * to initialize the minimum distance to a very high value.
                 */

                // --- INPUT DATA ---
                const int x1 = positionXX[0];
                const int y1 = positionYY[0];

                // --- OUTPUT VARIABLE ---
                uint8_t identified_idx;

                // --- SEARCH LOGIC ---

                // Initialize the minimum distance to the highest possible float value.
                float min_dist_sq = FLT_MAX;
                uint8_t best_found_idx = 0;

                // Iterate through all 4 vertices
                for (uint8_t i = 0; i < 4; i++) {
                    const float dx = (float)x1 - FinalX[i];
                    const float dy = (float)y1 - FinalY[i];
                    const float dist_sq = dx * dx + dy * dy;

                    // The if is now simpler. It works for both the first and subsequent ones.
                    // - In the first cycle: dist_sq will ALWAYS be < FLT_MAX.
                    // - In subsequent cycles: it works as a normal comparison.
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        best_found_idx = i;
                    }
                }

                identified_idx = best_found_idx;

                // --- SAFETY CHECK (STILL VALID AND IMPORTANT) ---
                const float MAX_ALLOWED_DISTANCE_SQ = 200.0f * 200.0f;
                if (min_dist_sq > MAX_ALLOWED_DISTANCE_SQ) {
                    // Point too far, likely noise. Do nothing to avoid jumps.
                    return;
                }

                // --- PHASE 2: RECONSTRUCTION VIA RIGID DRAGGING ---

                // Calculate the displacement (delta) of only the identified point
                const float delta_x = (float)x1 - FinalX[identified_idx];
                const float delta_y = (float)y1 - FinalY[identified_idx];

                // Apply the same delta to all 4 vertices to drag the rectangle
                for (uint8_t i = 0; i < 4; i++) {
                    FinalX[i] += delta_x;
                    FinalY[i] += delta_y;
                    positionXX[i] = FinalX[i];
                    positionYY[i] = FinalY[i];
                }
            } else {
                return;
            }
            // else: if prev_num_points_seen == 0, do nothing.
        }

    #ifdef COMMENTO
        if (num_points_seen == 2) {
            // ==================================================================================
            // PHASE 1 - Final Version with Normalized Costs and Dynamic Angle
            // ==================================================================================

            const int x1 = positionXX[0], y1 = positionYY[0];
            const int x2 = positionXX[1], y2 = positionYY[1];

            uint8_t best_idx1 = 0, best_idx2 = 1;
            float min_total_cost = -1.0f;

            // --- 1. SETUP AND PRELIMINARY POSITION COST CALCULATION ---
            const uint8_t pairs[6][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}, {1, 2}};
            float pos_costs[12];
            for (int i = 0; i < 6; i++) {
                const uint8_t v1_idx = pairs[i][0], v2_idx = pairs[i][1];
                const float dx1_opt1 = (float)x1 - FinalX[v1_idx], dy1_opt1 = (float)y1 - FinalY[v1_idx];
                const float dx2_opt1 = (float)x2 - FinalX[v2_idx], dy2_opt1 = (float)y2 - FinalY[v2_idx];
                pos_costs[i * 2] =
                    (dx1_opt1 * dx1_opt1 + dy1_opt1 * dy1_opt1) + (dx2_opt1 * dx2_opt1 + dy2_opt1 * dy2_opt1);
                const float dx1_opt2 = (float)x1 - FinalX[v2_idx], dy1_opt2 = (float)y1 - FinalY[v2_idx];
                const float dx2_opt2 = (float)x2 - FinalX[v1_idx], dy2_opt2 = (float)y2 - FinalY[v1_idx];
                pos_costs[i * 2 + 1] =
                    (dx1_opt2 * dx1_opt2 + dy1_opt2 * dy1_opt2) + (dx2_opt2 * dx2_opt2 + dy2_opt2 * dy2_opt2);
            }

            // --- 2. SETUP FOR NORMALIZATION ---
            // Shape template
            const float d2_expected_side_h = height * height;
            const float expected_width = height * ideal_aspect_ratio;
            const float d2_expected_side_w = expected_width * expected_width;
            const float d2_expected_diag = d2_expected_side_w + d2_expected_side_h;
            const float expected_d2s[6] = {d2_expected_side_w, d2_expected_side_w, d2_expected_side_h,
                                           d2_expected_side_h, d2_expected_diag,   d2_expected_diag};

            // Normalization factor for position
            const float pos_normalization_factor = (width * width) + (height * height) + 1e-6f;

            // Weights (starting values to calibrate)
            const float W_POS = 1.0f;    // Weight for position cost
            const float W_SHAPE = 1.5f;  // We give a bit more importance to shape
            const float W_ANGLE = 0.8f;  // We give a bit less importance to angle

            // --- 3. CALCULATION OF MEASURED VALUES ---
            const float dx_measured = (float)x1 - (float)x2;
            const float dy_measured = (float)y1 - (float)y2;
            const float d2_measured = dx_measured * dx_measured + dy_measured * dy_measured;
            const float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
            const float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
            const float dot_product = cdx1 * cdx2 + cdy1 * cdy2;
            const float mag_prod = sqrtf((cdx1 * cdx1 + cdy1 * cdy1) * (cdx2 * cdx2 + cdy2 * cdy2));
            const float cos_theta = dot_product / (mag_prod + 1e-6f);

            // --- 4. FINAL DECISION CYCLE WITH NORMALIZED COSTS AND DYNAMIC ANGLE ---

            // Dynamic calculation of expected cosines based on aspect ratio
            const float r = ideal_aspect_ratio;
            const float r2 = r * r;
            // Cosine of the angle for "long" sides (associated with width)
            const float cos_theta_expected_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
            // Cosine of the angle for "short" sides (associated with height)
            const float cos_theta_expected_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);

            for (int i = 0; i < 6; i++) {
                // A) Normalized Angular Cost with DYNAMIC expected value
                float cos_theta_expected;
                // The convention used in 'expected_d2s' associates the first 2 indices to width (W)
                // and the next 2 to height (H). We use the same convention here.
                if (i < 2) {
                    cos_theta_expected = cos_theta_expected_W;
                } else if (i < 4) {
                    cos_theta_expected = cos_theta_expected_H;
                } else {
                    cos_theta_expected = -1.0f;
                }  // Diagonals

                const float diff_cos = cos_theta - cos_theta_expected;
                const float angle_cost_norm = (diff_cos * diff_cos) / 4.0f;  // Normalized between [0, 1]

                // B) Normalized Shape Cost
                const float shape_cost_norm = fabsf(d2_measured - expected_d2s[i]) / (expected_d2s[i] + 1e-6f);

                // C) Calculation of Total Cost for the two possible assignments
                const float cost1_norm = (pos_costs[i * 2] / pos_normalization_factor) * W_POS +
                                         shape_cost_norm * W_SHAPE + angle_cost_norm * W_ANGLE;
                const float cost2_norm = (pos_costs[i * 2 + 1] / pos_normalization_factor) * W_POS +
                                         shape_cost_norm * W_SHAPE + angle_cost_norm * W_ANGLE;

                // D) Update of minimum
                if (min_total_cost < 0 || cost1_norm < min_total_cost) {
                    min_total_cost = cost1_norm;
                    best_idx1 = pairs[i][0];
                    best_idx2 = pairs[i][1];
                }
                if (min_total_cost < 0 || cost2_norm < min_total_cost) {
                    min_total_cost = cost2_norm;
                    best_idx1 = pairs[i][1];
                    best_idx2 = pairs[i][0];
                }
            }

            // --- 5. ASSIGNMENT AND PREPARATION FOR PHASE 2 ---
            const uint8_t idx1 = best_idx1;
            const uint8_t idx2 = best_idx2;
            float Vx[4], Vy[4];
            Vx[idx1] = (float)x1;
            Vy[idx1] = (float)y1;
            Vx[idx2] = (float)x2;
            Vy[idx2] = (float)y2;

            // ... here continues PHASE 2 ...

            // ====================================================================================================
            // PHASE 2: GEOMETRIC RECONSTRUCTION
            // ====================================================================================================

            bool top = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
            bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
            bool left = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
            bool right = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

            if (top || bottom) {
                uint8_t p1_idx, p2_idx, p3_idx, p4_idx;
                if (top) {
                    p1_idx = 0;
                    p2_idx = 1;
                    p3_idx = 2;
                    p4_idx = 3;
                } else {
                    p1_idx = 2;
                    p2_idx = 3;
                    p3_idx = 0;
                    p4_idx = 1;
                }

                if (Vx[p1_idx] > Vx[p2_idx]) {
                    uint8_t tmp = p1_idx;
                    p1_idx = p2_idx;
                    p2_idx = tmp;
                }

                float dx = Vx[p2_idx] - Vx[p1_idx], dy = Vy[p2_idx] - Vy[p1_idx];
                float base = hypotf(dx, dy);  // Measured width

                // The guard 'width < 1e-6f' is no longer needed, we trust the ideal ratio
                if (!(base < 1e-6f)) {
                    // --- CORRECT LOGIC ---
                    // Removed scaleFactor. We calculate the height directly from the measured width
                    // and our ideal and stable aspect ratio.
                    // Formula: Height = Width / (Width/Height)
                    float newHeight = base / ideal_aspect_ratio;

                    float inv_base = 1.0f / base;
                    float ndx = -dy * inv_base, ndy = dx * inv_base;

                    if (top) {
                        Vx[p3_idx] = Vx[p1_idx] + ndx * newHeight;
                        Vy[p3_idx] = Vy[p1_idx] + ndy * newHeight;
                        Vx[p4_idx] = Vx[p2_idx] + ndx * newHeight;
                        Vy[p4_idx] = Vy[p2_idx] + ndy * newHeight;
                    } else {
                        Vx[p3_idx] = Vx[p1_idx] - ndx * newHeight;
                        Vy[p3_idx] = Vy[p1_idx] - ndy * newHeight;
                        Vx[p4_idx] = Vx[p2_idx] - ndx * newHeight;
                        Vy[p4_idx] = Vy[p2_idx] - ndy * newHeight;
                    }
                }
            } else if (left || right) {
                uint8_t p1_idx, p2_idx, p3_idx, p4_idx;
                if (left) {
                    p1_idx = 0;
                    p2_idx = 2;
                    p3_idx = 1;
                    p4_idx = 3;
                } else {
                    p1_idx = 1;
                    p2_idx = 3;
                    p3_idx = 0;
                    p4_idx = 2;
                }

                if (Vy[p1_idx] > Vy[p2_idx]) {
                    uint8_t tmp = p1_idx;
                    p1_idx = p2_idx;
                    p2_idx = tmp;
                }

                float dx = Vx[p2_idx] - Vx[p1_idx], dy = Vy[p2_idx] - Vy[p1_idx];
                float hgt = hypotf(dx, dy);  // Measured height

                // The guard 'height < 1e-6f' is no longer needed
                if (!(hgt < 1e-6f)) {
                    // --- CORRECT LOGIC ---
                    // Removed scaleFactor. We calculate the width directly from the measured height
                    // and our ideal and stable aspect ratio.
                    // Formula: Width = Height * (Width/Height)
                    float newWidth = hgt * ideal_aspect_ratio;

                    float inv_hgt = 1.0f / hgt;
                    float ndx = -dy * inv_hgt, ndy = dx * inv_hgt;

                    if (left) {
                        Vx[p3_idx] = Vx[p1_idx] - ndx * newWidth;
                        Vy[p3_idx] = Vy[p1_idx] - ndy * newWidth;
                        Vx[p4_idx] = Vx[p2_idx] - ndx * newWidth;
                        Vy[p4_idx] = Vy[p2_idx] - ndy * newWidth;
                    } else {
                        Vx[p3_idx] = Vx[p1_idx] + ndx * newWidth;
                        Vy[p3_idx] = Vy[p1_idx] + ndy * newWidth;
                        Vx[p4_idx] = Vx[p2_idx] + ndx * newWidth;
                        Vy[p4_idx] = Vy[p2_idx] + ndy * newWidth;
                    }
                }
            } else {
                // --- DIAGONAL BLOCK: CORRECT AND SAFE LOGIC ---
                bool diagAD = ((idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0));

                // Identification of start and end points
                float p_start_x = diagAD ? Vx[0] : Vx[1];
                float p_start_y = diagAD ? Vy[0] : Vy[1];
                float p_end_x = diagAD ? Vx[3] : Vx[2];
                float p_end_y = diagAD ? Vy[3] : Vy[2];

                // 1. Measurement of current diagonal
                float dx = p_end_x - p_start_x;
                float dy = p_end_y - p_start_y;
                float L = hypotf(dx, dy);

                if (!(L < 1e-6f)) {
                    // --- NEW LOGIC: Authoritative Aspect Ratio ---
                    // 2. We calculate W and H directly from the measured diagonal (L) and the ideal ratio
                    float ratio_sq_plus_one = ideal_aspect_ratio * ideal_aspect_ratio + 1.0f;
                    float H = L / sqrtf(ratio_sq_plus_one);
                    float W = H * ideal_aspect_ratio;

                    // 3. Calculation of the rotation angle using the standard method
                    // Angle of the diagonal we measured
                    float phi_measured = atan2f(dy, dx);

                    // Angle that the diagonal SHOULD have, using the new W and H
                    float phi_ideal = diagAD ? atan2f(H, W) : atan2f(H, -W);

                    // The rotation to apply is the difference between the two angles
                    float theta = phi_measured - phi_ideal;
                    float c = cosf(theta);
                    float si = sinf(theta);

                    // 4. Reconstruction of vertices (identical to before, but with corrected values)
                    // Vectors of sides, rotated and scaled
                    float wvx = c * W, wvy = si * W;
                    float hvx = -si * H, hvy = c * H;

                    if (diagAD) {
                        Vx[1] = Vx[0] + wvx;
                        Vy[1] = Vy[0] + wvy;
                        Vx[2] = Vx[0] + hvx;
                        Vy[2] = Vy[0] + hvy;
                    } else {  // diagBC
                        Vx[0] = Vx[1] - wvx;
                        Vy[0] = Vy[1] - wvy;
                        Vx[3] = Vx[1] + hvx;
                        Vy[3] = Vy[1] + hvy;
                    }
                }
            }
            // ====================================================================================================
            // PHASE 3: SMOOTHING AND FINAL UPDATE
            // ====================================================================================================

            const float smoothFactor = 0.7f;
            for (int i = 0; i < 4; i++) {
                FinalX[i] = FinalX[i] * smoothFactor + Vx[i] * (1.0f - smoothFactor);
                FinalY[i] = FinalY[i] * smoothFactor + Vy[i] * (1.0f - smoothFactor);
                positionXX[i] = (int)roundf(FinalX[i]);
                positionYY[i] = (int)roundf(FinalY[i]);
            }

            current_point_seen_mask = (1 << (3 - idx1)) | (1 << (3 - idx2));
        }
    #endif  // COMMENTO
    #ifdef COMMENTO
        // last chatgpt good
        if (num_points_seen == 2) {
            const int x1 = positionXX[0], y1 = positionYY[0];
            const int x2 = positionXX[1], y2 = positionYY[1];

            uint8_t best_idx1 = 0, best_idx2 = 1;
            float min_total_cost = -1.0f;

            const uint8_t pairs[6][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}, {1, 2}};
            float pos_costs[12];

            for (int i = 0; i < 6; i++) {
                const uint8_t v1 = pairs[i][0], v2 = pairs[i][1];

                float dx1a = (float)x1 - (float)FinalX[v1];
                float dy1a = (float)y1 - (float)FinalY[v1];
                float dx2a = (float)x2 - (float)FinalX[v2];
                float dy2a = (float)y2 - (float)FinalY[v2];
                pos_costs[i * 2] = dx1a * dx1a + dy1a * dy1a + dx2a * dx2a + dy2a * dy2a;

                float dx1b = (float)x1 - (float)FinalX[v2];
                float dy1b = (float)y1 - (float)FinalY[v2];
                float dx2b = (float)x2 - (float)FinalX[v1];
                float dy2b = (float)y2 - (float)FinalY[v1];
                pos_costs[i * 2 + 1] = dx1b * dx1b + dy1b * dy1b + dx2b * dx2b + dy2b * dy2b;
            }

            const float d2H = height * height;
            const float estW = height * ideal_aspect_ratio;
            const float d2W = estW * estW;
            const float d2D = d2W + d2H;
            const float expected_d2s[6] = {d2W, d2W, d2H, d2H, d2D, d2D};

            const float pos_norm = (width * width + height * height + 1e-6f);

            const float W_POS = 1.0f, W_SHAPE = 1.5f, W_ANGLE = 0.8f;

            float dxm = (float)x1 - (float)x2;
            float dym = (float)y1 - (float)y2;
            float d2m = dxm * dxm + dym * dym;

            float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
            float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
            float dot = cdx1 * cdx2 + cdy1 * cdy2;
            float mag = sqrtf((cdx1 * cdx1 + cdy1 * cdy1) * (cdx2 * cdx2 + cdy2 * cdy2));
            mag = fmaxf(mag, 1e-6f);
            float cos_theta = dot / mag;
            cos_theta = fminf(1.0f, fmaxf(-1.0f, cos_theta));

            const float r = ideal_aspect_ratio;
            const float r2 = r * r;
            const float cos_theta_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
            const float cos_theta_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);

            for (int i = 0; i < 6; i++) {
                float expected_cos = (i < 2) ? cos_theta_W : (i < 4) ? cos_theta_H : -1.0f;
                float diff_cos = cos_theta - expected_cos;
                float angle_cost = (diff_cos * diff_cos) / 4.0f;

                float safe_d2 = fmaxf(expected_d2s[i], 25.0f);
                float shape_cost = fabsf(d2m - expected_d2s[i]) / safe_d2;

                float cost1 = (pos_costs[i * 2] / pos_norm) * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;
                float cost2 = (pos_costs[i * 2 + 1] / pos_norm) * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;

                uint8_t i1 = pairs[i][0], i2 = pairs[i][1];
                float best_cost = cost1;

                if (cost2 < best_cost) {
                    best_cost = cost2;
                    i1 = pairs[i][1];
                    i2 = pairs[i][0];
                }

                if (min_total_cost < 0 || best_cost < min_total_cost) {
                    min_total_cost = best_cost;
                    best_idx1 = i1;
                    best_idx2 = i2;
                }
            }

            // ==================================================================================
            // PHASE 2 - Geometric reconstruction with top/bottom perspective correction
            // ==================================================================================

            float Vx[4], Vy[4];
            for (int i = 0; i < 4; i++) {
                Vx[i] = (float)FinalX[i];
                Vy[i] = (float)FinalY[i];
            }
            Vx[best_idx1] = (float)x1;
            Vy[best_idx1] = (float)y1;
            Vx[best_idx2] = (float)x2;
            Vy[best_idx2] = (float)y2;

            const uint8_t idx1 = best_idx1, idx2 = best_idx2;
            bool top = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
            bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
            bool left = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
            bool right = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

            if (top || bottom) {
                uint8_t p1 = top ? 0 : 2;
                uint8_t p2 = top ? 1 : 3;
                uint8_t p3 = top ? 2 : 0;
                uint8_t p4 = top ? 3 : 1;

                if (Vx[p1] > Vx[p2]) {
                    uint8_t t = p1;
                    p1 = p2;
                    p2 = t;
                }

                float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
                float base = sqrtf(dx * dx + dy * dy);
                float cos_angle = fabsf(dx) / fmaxf(base, 1e-6f);
                base = base / fmaxf(cos_angle, 0.3f);

                if (base > 2.0f) {
                    float newH = base / ideal_aspect_ratio;
                    newH = fmaxf(newH, height * 0.5f);

                    float inv_base = 1.0f / base;
                    float ndx = -dy * inv_base, ndy = dx * inv_base;

                    float sx = ndx * newH, sy = ndy * newH;

                    if (top) {
                        Vx[p3] = Vx[p1] + sx;
                        Vy[p3] = Vy[p1] + sy;
                        Vx[p4] = Vx[p2] + sx;
                        Vy[p4] = Vy[p2] + sy;
                    } else {
                        Vx[p3] = Vx[p1] - sx;
                        Vy[p3] = Vy[p1] - sy;
                        Vx[p4] = Vx[p2] - sx;
                        Vy[p4] = Vy[p2] - sy;
                    }
                }
            } else if (left || right) {
                uint8_t p1 = left ? 0 : 1;
                uint8_t p2 = left ? 2 : 3;
                uint8_t p3 = left ? 1 : 0;
                uint8_t p4 = left ? 3 : 2;

                if (Vy[p1] > Vy[p2]) {
                    uint8_t t = p1;
                    p1 = p2;
                    p2 = t;
                }

                float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
                float hgt = sqrtf(dx * dx + dy * dy);

                if (hgt > 2.0f) {
                    float newW = hgt * ideal_aspect_ratio;
                    float inv_hgt = 1.0f / hgt;
                    float ndx = -dy * inv_hgt, ndy = dx * inv_hgt;

                    float sx = ndx * newW, sy = ndy * newW;

                    if (left) {
                        Vx[p3] = Vx[p1] - sx;
                        Vy[p3] = Vy[p1] - sy;
                        Vx[p4] = Vx[p2] - sx;
                        Vy[p4] = Vy[p2] - sy;
                    } else {
                        Vx[p3] = Vx[p1] + sx;
                        Vy[p3] = Vy[p1] + sy;
                        Vx[p4] = Vx[p2] + sx;
                        Vy[p4] = Vy[p2] + sy;
                    }
                }
            } else {
                bool diagAD = (idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0);

                float px1 = diagAD ? Vx[0] : Vx[1];
                float py1 = diagAD ? Vy[0] : Vy[1];
                float px2 = diagAD ? Vx[3] : Vx[2];
                float py2 = diagAD ? Vy[3] : Vy[2];

                float dx = px2 - px1, dy = py2 - py1;
                float L = sqrtf(dx * dx + dy * dy);

                if (L > 2.0f) {
                    float ratio_sq = ideal_aspect_ratio * ideal_aspect_ratio + 1.0f;
                    float H = L / sqrtf(ratio_sq);
                    float W = H * ideal_aspect_ratio;

                    float phi_m = atan2f(dy, dx);
                    float phi_i = diagAD ? atan2f(H, W) : atan2f(H, -W);
                    float theta = phi_m - phi_i;

                    float c = cosf(theta), s = sinf(theta);
                    float wx = c * W, wy = s * W;
                    float hx = -s * H, hy = c * H;

                    if (diagAD) {
                        Vx[1] = Vx[0] + wx;
                        Vy[1] = Vy[0] + wy;
                        Vx[2] = Vx[0] + hx;
                        Vy[2] = Vy[0] + hy;
                    } else {
                        Vx[0] = Vx[1] - wx;
                        Vy[0] = Vy[1] - wy;
                        Vx[3] = Vx[1] + hx;
                        Vy[3] = Vy[1] + hy;
                    }
                }
            }

            // ==================================================================================
            // PHASE 3 - Final assignment without smoothing
            // ==================================================================================

            for (int i = 0; i < 4; i++) {
                FinalX[i] = (int)roundf(Vx[i]);
                FinalY[i] = (int)roundf(Vy[i]);
                positionXX[i] = FinalX[i];
                positionYY[i] = FinalY[i];
            }

            current_point_seen_mask = (1 << (3 - best_idx1)) | (1 << (3 - best_idx2));
        }
    #endif

        // =================================================================================================
        // 2 POINTS SEEN MANAGEMENT (FINAL UNIFIED VERSION WITH 3-LEVEL WEIGHTS)
        //
        // Strategia:
        // La struttura di calcolo è unica. L'intelligenza è demandata alla selezione
        // dei pesi (W_POS, W_SHAPE, W_ANGLE) che vengono impostati a zero per i costi
        // che si vogliono escludere in un determinato contesto.
        // =================================================================================================

        if (num_points_seen == 2) {
            // --- FASE 1: IDENTIFICAZIONE ---

            const int x1 = positionXX[0], y1 = positionYY[0];
            const int x2 = positionXX[1], y2 = positionYY[1];

            uint8_t best_idx1 = 0, best_idx2 = 1;
            float min_total_cost = -1.0f;

            // --- Calcolo di TUTTI i possibili componenti di costo ---

            // 1. Costo di Posizione
            const uint8_t pairs[6][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}, {1, 2}};
            float pos_costs[12];
            for (int i = 0; i < 6; i++) {
                const uint8_t v1 = pairs[i][0], v2 = pairs[i][1];
                float dx1a = (float)x1 - FinalX[v1], dy1a = (float)y1 - FinalY[v1];
                float dx2a = (float)x2 - FinalX[v2], dy2a = (float)y2 - FinalY[v2];
                pos_costs[i * 2] = dx1a * dx1a + dy1a * dy1a + dx2a * dx2a + dy2a * dy2a;
                float dx1b = (float)x1 - FinalX[v2], dy1b = (float)y1 - FinalY[v2];
                float dx2b = (float)x2 - FinalX[v1], dy2b = (float)y2 - FinalY[v1];
                pos_costs[i * 2 + 1] = dx1b * dx1b + dy1b * dy1b + dx2b * dx2b + dy2b * dy2b;
            }
            const float inv_pos_norm = 1.0f / (width * width + height * height + 1e-6f);

            // 2. Costo di Forma e Angolo (calcolati anche se poi non usati)
            const float d2H = height * height;
            const float estW = height * ideal_aspect_ratio;
            const float d2W = estW * estW;
            const float d2D = d2W + d2H;
            const float expected_d2s[6] = {d2W, d2W, d2H, d2H, d2D, d2D};
            const float dxm = (float)x1 - (float)x2;
            const float dym = (float)y1 - (float)y2;
            const float d2m = dxm * dxm + dym * dym;
            const float r = ideal_aspect_ratio;
            const float r2 = r * r;
            const float cos_theta_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
            const float cos_theta_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);
            const float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
            const float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
            const float dot = cdx1 * cdx2 + cdy1 * cdy2;
            float mag = sqrtf((cdx1 * cdx1 + cdy1 * cdy1) * (cdx2 * cdx2 + cdy2 * cdy2));
            mag = fmaxf(mag, 1e-6f);
            float cos_theta = dot / mag;
            cos_theta = fminf(1.0f, fmaxf(-1.0f, cos_theta));

    #ifdef COMMENTO
            // --- Selezione dei pesi a 3 livelli (il "cervello" della logica) ---
            float W_POS, W_SHAPE, W_ANGLE;
            if (prev_num_points_seen <= 1) {
                // LIVELLO 3 (Riacquisizione): Usa SOLO la posizione.
                W_POS = 0.4f;
                W_SHAPE = 1.0f;
                W_ANGLE = 0.2f;
            } else if (prev_num_points_seen == 3) {
                // LIVELLO 2 (Transizione): Posizione debole, forma e angolo forti.
                W_POS = 0.15f;
                W_SHAPE = 1.0f;  // 1.5f;
                W_ANGLE = 0.8f;  // 1.0f;
            } else {             // prev_num_points_seen == 2 || 4
                // LIVELLO 1 (Stabile): Pesi bilanciati.
                W_POS = 1.0f;
                W_SHAPE = 0.3f;   // 1.5f;
                W_ANGLE = 0.15f;  // 0.8f;
            }
    #endif

            // =========================================================================================
            // DEFINITIVE MODEL v2 - Final Balancing
            // Synthesis between temporal stability and geometric robustness.
            // =========================================================================================

            float W_POS, W_SHAPE, W_ANGLE;

            if (prev_num_points_seen == 4) {
                // LIVELLO 1 (4/4): GOLD. Posizione e Forma sono ugualmente importanti.
                W_POS = 1.2f;
                W_SHAPE = 1.2f;  // <- Raised to match position.
                W_ANGLE = 1.0f;

            } else if (prev_num_points_seen == 3) {
                // LIVELLO 2 (3/4): SILVER. Leggermente meno fiducia nella posizione,
                // ma fiducia massima nella forma "ideale" della memoria.
                W_POS = 1.0f;
                W_SHAPE = 1.4f;  // <- Slightly higher than W_POS for shape perfection.
                W_ANGLE = 1.1f;

            } else if (prev_num_points_seen == 2) {
                // LIVELLO 3 (2/4): BRONZE. Meno fiducia nella posizione (basata su 2pt),
                // più fiducia nella geometria misurata ORA.
                W_POS = 0.8f;
                W_SHAPE = 1.3f;  // <- High to compensate for reduced confidence in W_POS.
                W_ANGLE = 1.2f;

            } else {  // prev_num_points_seen <= 1
                // LIVELLO 4 (1/4 o 0/4): INAFFIDABILE. La posizione non conta quasi nulla.
                // Ci si affida solo alla geometria dell'input corrente.
                W_POS = 0.1f;
                W_SHAPE = 1.8f;
                W_ANGLE = 0.1f;  // 05f; //0.1f; //1.5f;
            }

            // --- Calcolo del costo totale con un'unica logica ---
            for (int i = 0; i < 6; i++) {
                // const float shape_cost = fabsf(d2m - expected_d2s[i]) / fmaxf(expected_d2s[i], 25.0f);
                const float shape_cost = fabsf(d2m - expected_d2s[i]) / fmaxf(d2m, fmaxf(expected_d2s[i], 1.0f));

                const float expected_cos = (i < 2) ? cos_theta_W : (i < 4) ? cos_theta_H : -1.0f;
                const float diff_cos = cos_theta - expected_cos;
                const float angle_cost = (diff_cos * diff_cos) / 4.0f;
                const float normalized_pos_cost1 = pos_costs[i * 2] * inv_pos_norm;
                const float normalized_pos_cost2 = pos_costs[i * 2 + 1] * inv_pos_norm;

                const float cost1 = normalized_pos_cost1 * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;
                const float cost2 = normalized_pos_cost2 * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;

                if (min_total_cost < 0 || cost1 < min_total_cost) {
                    min_total_cost = cost1;
                    best_idx1 = pairs[i][0];
                    best_idx2 = pairs[i][1];
                }
                if (min_total_cost < 0 || cost2 < min_total_cost) {
                    min_total_cost = cost2;
                    best_idx1 = pairs[i][1];
                    best_idx2 = pairs[i][0];
                }
            }

            // --- PHASE 2: GEOMETRIC RECONSTRUCTION ---

            float Vx[4], Vy[4];
            for (int i = 0; i < 4; i++) {
                Vx[i] = FinalX[i];
                Vy[i] = FinalY[i];
            }
            Vx[best_idx1] = (float)x1;
            Vy[best_idx1] = (float)y1;
            Vx[best_idx2] = (float)x2;
            Vy[best_idx2] = (float)y2;

            const uint8_t idx1 = best_idx1, idx2 = best_idx2;
            const bool top = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
            const bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
            const bool left = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
            const bool right = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

            if (top || bottom) {
                uint8_t p1 = top ? 0 : 2, p2 = top ? 1 : 3;
                uint8_t p3 = top ? 2 : 0, p4 = top ? 3 : 1;
                if (Vx[p1] > Vx[p2]) {
                    uint8_t t = p1;
                    p1 = p2;
                    p2 = t;
                }
                float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
                float base = sqrtf(dx * dx + dy * dy);
                if (base > 2.0f) {
                    float cos_angle = fabsf(dx) / base;
                    base = base / fmaxf(cos_angle, 0.3f);
                    float newH = fmaxf(base / ideal_aspect_ratio, height * 0.5f);
                    float inv_base = 1.0f / base;
                    float ndx = -dy * inv_base, ndy = dx * inv_base;
                    float sx = ndx * newH, sy = ndy * newH;
                    if (top) {
                        Vx[p3] = Vx[p1] + sx;
                        Vy[p3] = Vy[p1] + sy;
                        Vx[p4] = Vx[p2] + sx;
                        Vy[p4] = Vy[p2] + sy;
                    } else {
                        Vx[p3] = Vx[p1] - sx;
                        Vy[p3] = Vy[p1] - sy;
                        Vx[p4] = Vx[p2] - sx;
                        Vy[p4] = Vy[p2] - sy;
                    }
                }
            } else if (left || right) {
                uint8_t p1 = left ? 0 : 1, p2 = left ? 2 : 3;
                uint8_t p3 = left ? 1 : 0, p4 = left ? 3 : 2;
                if (Vy[p1] > Vy[p2]) {
                    uint8_t t = p1;
                    p1 = p2;
                    p2 = t;
                }
                float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
                float hgt = sqrtf(dx * dx + dy * dy);
                if (hgt > 2.0f) {
                    float newW = hgt * ideal_aspect_ratio;
                    float inv_hgt = 1.0f / hgt;
                    float ndx = -dy * inv_hgt, ndy = dx * inv_hgt;
                    float sx = ndx * newW, sy = ndy * newW;
                    if (left) {
                        Vx[p3] = Vx[p1] - sx;
                        Vy[p3] = Vy[p1] - sy;
                        Vx[p4] = Vx[p2] - sx;
                        Vy[p4] = Vy[p2] - sy;
                    } else {
                        Vx[p3] = Vx[p1] + sx;
                        Vy[p3] = Vy[p1] + sy;
                        Vx[p4] = Vx[p2] + sx;
                        Vy[p4] = Vy[p2] + sy;
                    }
                }
            } else {  // Diagonale
                bool diagAD = (idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0);
                float px1 = diagAD ? Vx[0] : Vx[1], py1 = diagAD ? Vy[0] : Vy[1];
                float px2 = diagAD ? Vx[3] : Vx[2], py2 = diagAD ? Vy[3] : Vy[2];
                float dx = px2 - px1, dy = py2 - py1;
                float L = sqrtf(dx * dx + dy * dy);
                if (L > 2.0f) {
                    float H = L / sqrtf(ideal_aspect_ratio * ideal_aspect_ratio + 1.0f);
                    float W = H * ideal_aspect_ratio;
                    float phi_m = atan2f(dy, dx);
                    float phi_i = diagAD ? atan2f(H, W) : atan2f(H, -W);
                    float theta = phi_m - phi_i;
                    float c = cosf(theta), s = sinf(theta);
                    float wx = c * W, wy = s * W;
                    float hx = -s * H, hy = c * H;
                    if (diagAD) {
                        Vx[1] = Vx[0] + wx;
                        Vy[1] = Vy[0] + wy;
                        Vx[2] = Vx[0] + hx;
                        Vy[2] = Vy[0] + hy;
                    } else {
                        Vx[0] = Vx[1] - wx;
                        Vy[0] = Vy[1] - wy;
                        Vx[3] = Vx[1] + hx;
                        Vy[3] = Vy[1] + hy;
                    }
                }
            }

            // --- PHASE 3: FINAL ASSIGNMENT ---

            for (int i = 0; i < 4; i++) {
                FinalX[i] = roundf(Vx[i]);
                FinalY[i] = roundf(Vy[i]);
                positionXX[i] = FinalX[i];
                positionYY[i] = FinalY[i];
            }
            current_point_seen_mask = (1 << (3 - best_idx1)) | (1 << (3 - best_idx2));
        }

        else if (num_points_seen == 3) {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // ==================== logica per 3 punti disordinati ============================================== //
            ////////////////////////////////////////////////////////////////////////////////////////////////////////

            // If we see 3 points, we perform the simple and fast estimation of the 4th point.
            int32_t d01_sq, d12_sq, d02_sq;

            dx = positionXX[0] - positionXX[1];
            dy = positionYY[0] - positionYY[1];
            d01_sq = dx * dx + dy * dy;

            dx = positionXX[1] - positionXX[2];
            dy = positionYY[1] - positionYY[2];
            d12_sq = dx * dx + dy * dy;

            dx = positionXX[0] - positionXX[2];
            dy = positionYY[0] - positionYY[2];
            d02_sq = dx * dx + dy * dy;

            uint8_t a_idx;  // one of the two points of the longest side of the triangle that corresponds to a diagonal
                            // of the rectangle
            uint8_t b_idx;  // vertex between long side and short side of the triangle
            uint8_t c_idx;  // one of the two points of the longest side of the triangle that corresponds to a diagonal
                            // of the rectangle

            if (d01_sq >= d12_sq && d01_sq >= d02_sq) {
                // d01 is the diagonal of the rectangle
                // b, the point between long side and short side is necessarily point 2
                a_idx = 0;
                c_idx = 1;
                b_idx = 2;
                // current_point_seen_mask = 0b00001111;
            } else if (d12_sq >= d02_sq) {
                // d12 is the diagonal of the rectangle
                // b, the point between long side and short side is necessarily point 0
                a_idx = 1;
                c_idx = 2;
                b_idx = 0;
                // current_point_seen_mask = 0b00001111;
            } else {
                // d02 is the diagonal of the rectangle
                // b, the point between long side and short side is necessarily point 2
                a_idx = 0;
                c_idx = 2;
                b_idx = 1;
                // current_point_seen_mask = 0b00001111;
            }

            // using the properties of the Parallelogram I calculate the missing 4th vertex,
            // without however knowing its exact logical position, which will be identified later by the management of
            // the 4 points
            positionXX[3] = positionXX[a_idx] + positionXX[c_idx] - positionXX[b_idx];
            positionYY[3] = positionYY[a_idx] + positionYY[c_idx] - positionYY[b_idx];
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ==================== logic for 4 unordered points ============================================== //
        ////////////////////////////////////////////////////////////////////////////////////////////////////////

        // --- PHASE 3: Final Ordering of the 4 Points ---
        // This section is always executed, to ensure that FinalX/Y
        // always have a consistent order A,B,C,D.

        uint8_t orderX[4] = {0, 1, 2, 3};
        uint8_t orderY[4] = {0, 1, 2, 3};

        // more verbose ordering, but more performant
        // Ordering of indices based on X coordinate (optimized sorting network)
        {
            uint8_t tmp;
            if (positionXX[orderX[0]] > positionXX[orderX[1]]) {
                tmp = orderX[0];
                orderX[0] = orderX[1];
                orderX[1] = tmp;
            }
            if (positionXX[orderX[2]] > positionXX[orderX[3]]) {
                tmp = orderX[2];
                orderX[2] = orderX[3];
                orderX[3] = tmp;
            }
            if (positionXX[orderX[0]] > positionXX[orderX[2]]) {
                tmp = orderX[0];
                orderX[0] = orderX[2];
                orderX[2] = tmp;
            }
            if (positionXX[orderX[1]] > positionXX[orderX[3]]) {
                tmp = orderX[1];
                orderX[1] = orderX[3];
                orderX[3] = tmp;
            }
            if (positionXX[orderX[1]] > positionXX[orderX[2]]) {
                tmp = orderX[1];
                orderX[1] = orderX[2];
                orderX[2] = tmp;
            }
        }

        // more verbose ordering, but more performant
        // Ordering of indices based on Y coordinate (optimized sorting network)
        {
            uint8_t tmp;
            if (positionYY[orderY[0]] > positionYY[orderY[1]]) {
                tmp = orderY[0];
                orderY[0] = orderY[1];
                orderY[1] = tmp;
            }
            if (positionYY[orderY[2]] > positionYY[orderY[3]]) {
                tmp = orderY[2];
                orderY[2] = orderY[3];
                orderY[3] = tmp;
            }
            if (positionYY[orderY[0]] > positionYY[orderY[2]]) {
                tmp = orderY[0];
                orderY[0] = orderY[2];
                orderY[2] = tmp;
            }
            if (positionYY[orderY[1]] > positionYY[orderY[3]]) {
                tmp = orderY[1];
                orderY[1] = orderY[3];
                orderY[3] = tmp;
            }
            if (positionYY[orderY[1]] > positionYY[orderY[2]]) {
                tmp = orderY[1];
                orderY[1] = orderY[2];
                orderY[2] = tmp;
            }
        }

        // Assignment of vertices A,B,C,D via heuristics
        int32_t dist_sq1, dist_sq2;

        dx = positionXX[orderY[0]] - positionXX[orderX[0]];
        dy = positionYY[orderY[0]] - positionYY[orderX[0]];
        dist_sq1 = (dx * dx) + (dy * dy);

        dx = positionXX[orderY[3]] - positionXX[orderX[0]];
        dy = positionYY[orderY[3]] - positionYY[orderX[0]];
        dist_sq2 = (dx * dx) + (dy * dy);

        const int CRITICAL_ZONE =
            (30 * CamToMouseMult);  // 30 is a value tested in many situations and seems to work well

        if ((positionYY[orderY[1]] - positionYY[orderY[0]]) > CRITICAL_ZONE) {
            // Normal Case
            if (dist_sq1 < dist_sq2) {
                a = orderX[0];
                d = orderX[3];
                if (orderX[1] == orderY[3]) {
                    c = orderX[1];
                    b = orderX[2];
                } else {
                    b = orderX[1];
                    c = orderX[2];
                }
            } else {
                c = orderX[0];
                b = orderX[3];
                if (orderX[1] == orderY[3]) {
                    d = orderX[1];
                    a = orderX[2];
                } else {
                    a = orderX[1];
                    d = orderX[2];
                }
            }
        } else {
            // Critical Zone Case (almost vertical rectangle)
            a = orderY[0];
            b = orderY[1];
            c = orderY[2];
            d = orderY[3];
        }

        // this would be sufficient for the critical zone this further check, but since it has
        // a small computational weight, we always do it for edge cases and as greater robustness
        // Final correction to ensure the convention (A=TL, B=TR, C=BL, D=BR)
        {
            uint8_t aux_swap;
            if (positionYY[a] > positionYY[c]) {
                aux_swap = a;
                a = c;
                c = aux_swap;
            }
            if (positionYY[b] > positionYY[d]) {
                aux_swap = b;
                b = d;
                d = aux_swap;
            }
            if (positionXX[a] > positionXX[b]) {
                aux_swap = a;
                a = b;
                b = aux_swap;
            }
            if (positionXX[c] > positionXX[d]) {
                aux_swap = c;
                c = d;
                d = aux_swap;
            }
        }

        // --- PHASE 4: Final Assignment and Derived Calculations ---
        FinalX[A] = positionXX[a];
        FinalY[A] = positionYY[a];
        FinalX[B] = positionXX[b];
        FinalY[B] = positionYY[b];
        FinalX[C] = positionXX[c];
        FinalY[C] = positionYY[c];
        FinalX[D] = positionXX[d];
        FinalY[D] = positionYY[d];

        //////////////////////////////////////////////////////////////////////////////////////////
        // ==== calculation of medianX, medianY, height, width, angle ... for compatibility ========== //
        //////////////////////////////////////////////////////////////////////////////////////////

        // Centroid calculation with rounding (the +2 is a trick for rounding to the nearest integer)
        medianX = (FinalX[A] + FinalX[B] + FinalX[C] + FinalX[D] + 2) / 4;
        medianY = (FinalY[A] + FinalY[B] + FinalY[C] + FinalY[D] + 2) / 4;

        // Final calculations maintained for compatibility
        float yDistLeft = hypotf((float)FinalY[A] - FinalY[C], (float)FinalX[A] - FinalX[C]);    // length of side AC
        float yDistRight = hypotf((float)FinalY[B] - FinalY[D], (float)FinalX[B] - FinalX[D]);   // length of side BD
        float xDistTop = hypotf((float)FinalY[A] - FinalY[B], (float)FinalX[A] - FinalX[B]);     // length of side AB
        float xDistBottom = hypotf((float)FinalY[C] - FinalY[D], (float)FinalX[C] - FinalX[D]);  // length of side CD
        height = (yDistLeft + yDistRight) / 2.0f;
        width = (xDistTop + xDistBottom) / 2.0f;

        if (num_points_seen == 4) {
            // NUOVO: Aggiorna la nostra "verità assoluta" sulla forma
            if (height > 1e-6f) {  // Avoid division by zero
                ideal_aspect_ratio = width / height;
            }
        }

        angle = (atan2f((float)FinalY[A] - FinalY[B], (float)FinalX[B] - FinalX[A]) +
                 atan2f((float)FinalY[C] - FinalY[D], (float)FinalX[D] - FinalX[C])) /
                2.0f;

        is_tracking_stable = true;
        prev_point_seen_mask = current_point_seen_mask;
        if (num_points_seen >= 3)
            current_point_seen_mask = 0b00001111;
    } else {
        is_tracking_stable = false;
        prev_point_seen_mask = 0;
    }
    prev_num_points_seen = num_points_seen;
}

#endif  // USE_SQUARE_ADVANCED