#ifndef rebet__TOAD_CONSTS_HPP_
#define rebet__TOAD_CONSTS_HPP_

#include <math.h>

inline constexpr double STARTING_BUDGET = 30.0; // Arbitrary
inline constexpr double V8X_POWER_COST = 14.25; // Set to LatencyT4TensorRT10FP16 of model
inline constexpr double V8N_POWER_COST = 1.5;   // Set to LatencyT4TensorRT10FP16 of model
inline constexpr double V8X_ACCURACY = 53.75;   // Set to COCO mAP of model
inline constexpr double V8N_ACCURACY = 37.0;    // Set to COCO mAP of model

#endif