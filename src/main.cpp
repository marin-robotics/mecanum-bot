#include "main.h"
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Vision vision_sensor(3);

pros::vision_signature_s_t ring_signature =
	vision_sensor.signature_from_utility(1, -205, 863, 329, 7521, 8655, 8088, 3.000, 0);
pros::vision_signature_s_t red_mogus_signature =
	vision_sensor.signature_from_utility(2, 5105, 6761, 5933, 699, 1629, 1164, 3.000, 0);
pros::vision_signature_s_t blue_mogus_signature =
	vision_sensor.signature_from_utility(2, -3049, -2407, -2728, 9121, 10519, 9820, 3.000, 0);

pros::Motor front_left (11, MOTOR_GEARSET_18);
pros::Motor front_right (1, MOTOR_GEARSET_18, true);
pros::Motor back_left (20, MOTOR_GEARSET_18);
pros::Motor back_right (10, MOTOR_GEARSET_18, true);

void move_left(int speed) {
	front_left = speed;
	back_left = speed;
}

void move_right(int speed) {
	front_right = speed;
	back_right = speed;
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	vision_sensor.set_signature(ring_signature.id, &ring_signature);
	vision_sensor.set_signature(red_mogus_signature.id, &red_mogus_signature);
	vision_sensor.set_signature(blue_mogus_signature.id, &blue_mogus_signature);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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

	while (true) {
		pros::vision_object_s_t red_mobile_goals[2];
		vision_sensor.read_by_sig(0, blue_mogus_signature.id, 2, red_mobile_goals);

		pros::lcd::print(0, "object count: %d", vision_sensor.get_object_count());
		pros::lcd::print(1, "red object 0: (%u, %u)", red_mobile_goals[0].x_middle_coord, red_mobile_goals[0].y_middle_coord);
		pros::lcd::print(2, "red object 1: (%u, %u)", red_mobile_goals[1].x_middle_coord, red_mobile_goals[1].y_middle_coord);


		if (185 < red_mobile_goals[0].x_middle_coord && red_mobile_goals[0].x_middle_coord < 215) {
			move_left(50);
			move_right(50);
		}
		else{
			if (red_mobile_goals[0].x_middle_coord > 200) {
				move_left(50);
				move_right(0);
			}
			else{
				move_right(50);
				move_left(0);
			}

		}

		pros::delay(20);
	}
}

/**
 * Runs the operato control code. This function will be started in its own task
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
	int controlMode = 1;
	bool tanksteer = false;

	int cosMap[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		1,1,1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,5,5,5,6,6,7,7,8,8,9,9,10,10,11,
		12,12,13,14,15,15,16,17,18,19,19,20,21,22,23,24,25,26,27,28,30,31,
		32,33,34,36,37,38,40,41,42,44,45,47,48,50,51,53,55,56,58,60,61,63,
		65,67,69,70,72,74,76,78,80,82,84,87,89,91,93,95,97,100,102,104,107,
		109,112,114,116,119,121,124,127};

	int throttleMap[] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,
		10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,
		20,21,21,22,22,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,
		30,31,31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,
		40,41,41,42,42,44,45,47,48,50,51,53,55,56,58,60,61,63,65,67,
		69,70,72,74,76,78,80,82,84,87,89,91,93,95,97,100,102,104,107,
		109,112,114,116,119,121,124,127};

	while (true) {

		int left_x = master.get_analog(ANALOG_LEFT_X);
		int left_y = master.get_analog(ANALOG_LEFT_Y);

		int right_x = master.get_analog(ANALOG_RIGHT_X);
		int right_y = master.get_analog(ANALOG_RIGHT_Y);

		if (master.get_digital(DIGITAL_A)) {
			controlMode = 0; //cosmode
			master.print(0, 0, "cosmode");
		}

		if (master.get_digital(DIGITAL_B)) {
			controlMode = -1; // normalmode
			master.print(0, 0, "normalmode");
		}

		if (master.get_digital(DIGITAL_X)) {
			controlMode = 1; // newmode
			master.print(0, 0, "newmode");
		}

		if (master.get_digital(DIGITAL_UP)) {
			tanksteer = true;
		}

		if (master.get_digital(DIGITAL_DOWN)) {
			tanksteer = false;
		}

		if (controlMode == 0) {
			left_x = left_x < 0 ? -cosMap[-left_x] : cosMap[left_x];
			left_y = left_y < 0 ? -cosMap[-left_y] : cosMap[left_y];

			right_x = right_x < 0 ? -cosMap[-right_x] : cosMap[right_x];
			right_y = right_y < 0 ? -cosMap[-right_y] : cosMap[right_y];
		} else if (controlMode == 1) {
			left_x = left_x < 0 ? -throttleMap[-left_x] : throttleMap[left_x];
			left_y = left_y < 0 ? -throttleMap[-left_y] : throttleMap[left_y];

			right_x = right_x < 0 ? -throttleMap[-right_x] : throttleMap[right_x];
			right_y = right_y < 0 ? -throttleMap[-right_y] : throttleMap[right_y];
		}

		if (tanksteer) {
			move_left(left_y);
			move_right(right_y);
		} else {
			front_right = (right_y - right_x) - left_x;
			back_right = (right_y + right_x) - left_x;
			front_left = (right_y + right_x) + left_x;
			back_left = (right_y - right_x) + left_x;
		}


		pros::delay(20);
	}
}
