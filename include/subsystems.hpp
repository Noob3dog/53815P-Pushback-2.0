#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup strafe_mg({5});  // Strafe motors (replace port numbers as needed)
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');