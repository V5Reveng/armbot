#include <pros/adi.hpp>
#include <pros/misc.hpp>
#include <pros/motors.hpp>
#include <pros/rtos.hpp>

#include "main.hpp"

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

// #define allows it to be effectively any integral/floating-point type
#define MOTOR_MAX_VOLTAGE 127
#define CONTROLLER_MAX_ANALOG 127

class Arm {
protected:
	pros::Motor m_motor;

	static constexpr int32_t VELOCITY = MOTOR_MAX_VOLTAGE;  // the lift is geared down pretty significantly so we want to go as fast as possible
	static constexpr double OPEN_POSITION = 100.0;  // FIXME arbitrary
	static constexpr double CLOSED_POSITION = 0.0;
public:
	Arm(uint8_t const port) : m_motor{ port } {}

	void set_position(double const angle) {
		m_motor.move_absolute(std::clamp(angle, CLOSED_POSITION, OPEN_POSITION), VELOCITY);
	}
	void raise() {
		set_position(OPEN_POSITION);
	}
	void lower() {
		set_position(CLOSED_POSITION);
	}
	void stop_moving() {
		m_motor.move(0);
	}
};

class Claw {
protected:
	pros::Motor m_motor;
	bool m_is_open = false;

	static constexpr double OPEN_POSITION = 60.0;  // FIXME arbitrary
	static constexpr double CLOSED_POSITION = 0.0;
	static constexpr int32_t VELOCITY = MOTOR_MAX_VOLTAGE / 2;  // FIXME arbitrary
public:
	Claw(uint8_t const port) : m_motor{ port } {}
	~Claw() {
		set_open(false);
	}

	bool is_open() const {
		return m_is_open;
	}
	void set_open(bool const new_state) {
		m_is_open = new_state;
		reify_open_state();
	}
	void toggle_open() {
		set_open(!is_open());
	}
protected:
	void reify_open_state() {
		m_motor.move_absolute(m_is_open ? OPEN_POSITION : CLOSED_POSITION, VELOCITY);
	}
};

class Drivetrain {
protected:
	pros::ADIMotor m_front_left, m_front_right, m_back_left, m_back_right;
	static constexpr double DAMPENING = 1.0 / 3.0;  // FIXME
public:
	Drivetrain(uint8_t const front_left_port, uint8_t const front_right_port, uint8_t const back_left_port, uint8_t const back_right_port)
		: m_front_left{ front_left_port }, m_front_right{ front_right_port }, m_back_left{ back_left_port }, m_back_right{ back_right_port } {}

	void update(double const _forward_axis, double const _strafe_axis, double const _rotate_axis) {
		auto const forward_axis = transform_motor_power(_forward_axis);
		auto const strafe_axis = transform_motor_power(_strafe_axis);
		auto const rotate_axis = transform_motor_power(_rotate_axis);

		m_front_left.set_value(forward_axis + strafe_axis + rotate_axis);
		m_back_left.set_value(forward_axis - strafe_axis + rotate_axis);
		// right motors are inverted
		m_front_right.set_value(-(forward_axis - strafe_axis - rotate_axis));
		m_back_right.set_value(-(forward_axis + strafe_axis - rotate_axis));
	}
protected:
	static int32_t transform_motor_power(double const power_out_of_one) {
		return static_cast<int32_t>(power_out_of_one * power_out_of_one * power_out_of_one * MOTOR_MAX_VOLTAGE * DAMPENING);
	}
};

class Robot {
protected:
	pros::Controller controller{ pros::E_CONTROLLER_MASTER };
	Drivetrain drivetrain{ 1, 2, 3, 4 };  // FIXME arbitrary ports
	Claw claw{ 1 };  // FIXME arbitrary ports
	Arm arm{ 2 };  // FIXME arbitrary ports
public:
	Robot() {}

	void update() {
		update_drivetrain();
		update_arm();
		update_claw();
	}
protected:
	void update_claw() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			claw.toggle_open();
		}
	}
	void update_arm() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			arm.raise();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			arm.lower();
		} else {
			arm.stop_moving();
		}
	}
	void update_drivetrain() {
		auto const forward_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		auto const strafe_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		auto const rotate_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		static constexpr double OUT_OF_ONE_FACTOR = 1.0 / CONTROLLER_MAX_ANALOG;
		drivetrain.update(forward_axis * OUT_OF_ONE_FACTOR, strafe_axis * OUT_OF_ONE_FACTOR, rotate_axis * OUT_OF_ONE_FACTOR);
	}
};

void opcontrol() {
	Robot robot;

	while (true) {
		robot.update();
		pros::Task::delay(1);  // sched_yield
	}
}