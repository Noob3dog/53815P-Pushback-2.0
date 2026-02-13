#include "main.h"

// Define the global controller (declared `extern` in EZ-Template util.hpp)
pros::Controller master(pros::E_CONTROLLER_MASTER);

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, -2},     // Left Chassis Ports (negative port will reverse it!)
    {3, 4},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    3,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    343);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {

		int strafe = master.get_analog(ANALOG_LEFT_X); // update strafe each loop
		strafe_mg.move(strafe);
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp
    chassis.pid_tuner_disable();  // Start with the PID tuner disabled just in case
    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::MotorGroup left_mg({3, 4});    // Creates a motor group withreversed ports 19, 20, 11
	pros::MotorGroup right_mg({-1, -2});  // Creates a motor group with forwards ports 8, 10, 12
	pros::MotorGroup strafe_mg({5}); // strafe motor with port 5
	pros::MotorGroup intake_mg({-6, 7});  // Creates a motor group with forward ports 6, 7
	pros::MotorGroup intake_Roller_mg({7});  // Creates a motor group with forward ports 7
	pros::MotorGroup score2_mg({-8});
	pros::MotorGroup outtake_mg({-9});
	pros::MotorGroup drive_mg({3, 4, -1, -2});
	pros::ADIDigitalOut match_Load('H',LOW); // creates a digital output on port H for the matchload
	pros::ADIButton button('A');
	pros::ADIDigitalIn limit_switch('B'); // Limit switch on port B for counting
	pros::MotorGroup counting_mg({21});



  // This is preference to what you like to drive on
  	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold is generally better for driver control, but coast can be used for a more "floaty" feel




  pros::Motor motor1(1);
	pros::Motor motor2(2);
	pros::Motor motor3(3);
	pros::Motor motor4(4);
	pros::Motor motor5(5);
	pros::Motor motor6(6);
	pros::Motor motor7(7);
	pros::Motor motor8(8);

  bool prev_up = false;
	bool prev_left = false;

	int color = 0; // 0 = none,  1 = blue,  -1 = red
	int driveSpeed=127; // drive speed
	int scoreMotor=100; // defines the scoreMotor variable
	int intakeSpeed=100; // intake speed
	int scoreSpeed=100; // outtake speed


  while (true) {

		// Temperature toggle handling (independent of intake buttons)
		bool up = master.get_digital(DIGITAL_UP);
		static bool showTemps = false;
		static std::uint32_t lastTempUpdate = 0;
		const std::uint32_t tempInterval = 900; // ms (slower scroll)

    //temperature dysplaying code
if (up && !prev_up) { // rising edge -> toggle
			showTemps = !showTemps;
			if (!showTemps) {
				master.clear_line(0);
				master.clear_line(1);
			} else {
				// immediate first update
				lastTempUpdate = pros::millis();
				int t1 = motor1.get_temperature();
				int t2 = motor2.get_temperature();
				int t3 = motor3.get_temperature();
				int t4 = motor4.get_temperature();
				int t5 = motor5.get_temperature();
				int t6 = motor6.get_temperature();
				int t7 = motor7.get_temperature();
				int t8 = motor8.get_temperature();
				char buf0[64];
				char buf1[64];
				std::snprintf(buf0, sizeof(buf0), "M1:%dC M2:%dC M3:%dC M4:%dC", t1, t2, t3, t4);
				std::snprintf(buf1, sizeof(buf1), "M5:%dC M6:%dC M7:%dC M8:%dC", t5, t6, t7, t8);
				// initialize scrolling strings and state
				static std::string scroll0;
				static std::string scroll1;
				static int pos0 = 0;
				static int pos1 = 0;
				static int dir0 = 1;
				static int dir1 = 1;
				const int displayWidth = 15; // controller columns [0..14]

				scroll0 = std::string(buf0);
				scroll1 = std::string(buf1);

				if ((int)scroll0.size() <= displayWidth) {
					master.set_text(0, 0, scroll0.c_str());
				} else {
					int maxStart = (int)scroll0.size() - displayWidth;
					if (pos0 < 0) pos0 = 0;
					if (pos0 > maxStart) pos0 = maxStart;
					std::string out = scroll0.substr(pos0, displayWidth);
					master.set_text(0, 0, out.c_str());
					// prepare for next tick
					pos0 += dir0;
					if (pos0 >= maxStart) { pos0 = maxStart; dir0 = -1; }
					else if (pos0 <= 0) { pos0 = 0; dir0 = 1; }
				}

				if ((int)scroll1.size() <= displayWidth) {
					master.set_text(1, 0, scroll1.c_str());
				} else {
					int maxStart = (int)scroll1.size() - displayWidth;
					if (pos1 < 0) pos1 = 0;
					if (pos1 > maxStart) pos1 = maxStart;
					std::string out1 = scroll1.substr(pos1, displayWidth);
					master.set_text(1, 0, out1.c_str());
					pos1 += dir1;
					if (pos1 >= maxStart) { pos1 = maxStart; dir1 = -1; }
					else if (pos1 <= 0) { pos1 = 0; dir1 = 1; }
				}
			}
		}

		if (showTemps && (pros::millis() - lastTempUpdate >= tempInterval)) {
			int t1 = motor1.get_temperature();
			int t2 = motor2.get_temperature();
			int t3 = motor3.get_temperature();
			int t4 = motor4.get_temperature();
			int t5 = motor5.get_temperature();
			int t6 = motor6.get_temperature();
			int t7 = motor7.get_temperature();
			int t8 = motor8.get_temperature();
				char buf0[64];
				char buf1[64];
				std::snprintf(buf0, sizeof(buf0), "M1:%dC M2:%dC M3:%dC M4:%dC", t1, t2, t3, t4);
				std::snprintf(buf1, sizeof(buf1), "M5:%dC M6:%dC M7:%dC M8:%dC", t5, t6, t7, t8);
				static std::string scroll0;
				static std::string scroll1;
				static int pos0 = 0;
				static int pos1 = 0;
				static int dir0 = 1;
				static int dir1 = 1;
				const int displayWidth = 15; // controller columns [0..14]

				scroll0 = std::string(buf0);
				scroll1 = std::string(buf1);

				if ((int)scroll0.size() <= displayWidth) {
					master.set_text(0, 0, scroll0.c_str());
				} else {
					int maxStart = (int)scroll0.size() - displayWidth;
					if (pos0 < 0) pos0 = 0;
					if (pos0 > maxStart) pos0 = maxStart;
					std::string out = scroll0.substr(pos0, displayWidth);
					master.set_text(0, 0, out.c_str());
					pos0 += dir0;
					if (pos0 >= maxStart) { pos0 = maxStart; dir0 = -1; }
					else if (pos0 <= 0) { pos0 = 0; dir0 = 1; }
				}

				if ((int)scroll1.size() <= displayWidth) {
					master.set_text(1, 0, scroll1.c_str());
				} else {
					int maxStart = (int)scroll1.size() - displayWidth;
					if (pos1 < 0) pos1 = 0;
					if (pos1 > maxStart) pos1 = maxStart;
					std::string out1 = scroll1.substr(pos1, displayWidth);
					master.set_text(1, 0, out1.c_str());
					pos1 += dir1;
					if (pos1 >= maxStart) { pos1 = maxStart; dir1 = -1; }
					else if (pos1 <= 0) { pos1 = 0; dir1 = 1; }
				}
			lastTempUpdate = pros::millis();
		}

		prev_up = up;

    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
		// Outtake control
		if (master.get_digital(DIGITAL_R1)) {
			outtake_mg.move(scoreSpeed); // Move outtake forward
			pros::lcd::set_text(4, "Outtake Forward!");
		} else if (master.get_digital(DIGITAL_R2)) {
			outtake_mg.move(-scoreSpeed); // move motor backwards
			pros::lcd::set_text(4, "Outtake Reverse!");
		} else {
			outtake_mg.move(0); // stops outtake
			pros::lcd::set_text(4, "Outtake Stopped Color Mismatch.");
		}

		// Outtake control
		if (master.get_digital(DIGITAL_R1)) {
			outtake_mg.move(scoreSpeed); // Move outtake forward
			pros::lcd::set_text(4, "Outtake Forward!");
		} else if (master.get_digital(DIGITAL_R2)) {
			outtake_mg.move(-scoreSpeed); // move motor backwards
			pros::lcd::set_text(4, "Outtake Reverse!");
		} else {
			outtake_mg.move(0); // stops outtake
			pros::lcd::set_text(4, "Outtake Stopped Color Mismatch.");
		}

			//Intake/outtake controll
		if (master.get_digital(DIGITAL_L1)) // if the top left trigger is pressed
		{
			intake_mg.move(intakeSpeed); // Move outtake forward
			score2_mg.move(intakeSpeed);
			pros::lcd::set_text(4, "Intake Forward!");
		}
		else if (master.get_digital(DIGITAL_L2)) // if the bottom left trigger is pressed
		{
			intake_mg.move(-intakeSpeed); // move motor backwards
			score2_mg.move(-intakeSpeed);
			pros::lcd::set_text(4, "Intake Reverse!");
		}
		else // If none are pressed
		{
			intake_mg.move(0); // stops outtake
			score2_mg.move(0);
			pros::lcd::set_text(4, "Intake Stopped.");
		}


		if (master.get_digital(DIGITAL_LEFT))
		{
			intake_Roller_mg.move(intakeSpeed);
		}
		else if (master.get_digital(DIGITAL_A))
		{
			intake_Roller_mg.move(-intakeSpeed);
		}
		else
		{
			// If neither X nor B are pressed, only stop score2_mg when intake isn't active
			if (!master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2)) {
				intake_Roller_mg.move(0);
			}
		}


		if (master.get_digital(DIGITAL_DOWN))
		{
			score2_mg.move(intakeSpeed);
		}
		else if (master.get_digital(DIGITAL_B))
		{
			score2_mg.move(-intakeSpeed);
		}
		else
		{
			// If neither X nor B are pressed, only stop score2_mg when intake isn't active
			if (!master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2)) {
				score2_mg.move(0);
			}
		}

		
		// Match load control

	if (master.get_digital(DIGITAL_Y)) // if the top right trigger is pressed
		{
			match_Load.set_value(LOW); // Clamp
			pros::lcd::set_text(4, "Clamp!");
		}
		else if (master.get_digital(DIGITAL_RIGHT)) // if the bottom left trigger is pressed
		{
			match_Load.set_value(HIGH); // Unclamp
			pros::lcd::set_text(4, "Unclamp!");

		}


    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
