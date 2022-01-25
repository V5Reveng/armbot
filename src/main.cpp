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

class Claw {
protected:
	static constexpr double CLAMP_OPEN_POSITION = 60.0;  // FIXME arbitrary
	static constexpr double CLAMP_CLOSED_POSITION = 0.0;
	static constexpr int32_t CLAMP_VELOCITY = MOTOR_MAX_VOLTAGE / 2;  // FIXME arbitrary
	static constexpr int32_t LIFT_VELOCITY = MOTOR_MAX_VOLTAGE;  // the lift is geared down pretty significantly so we want to go as fast as possible
	static constexpr double LIFT_OPEN_POSITION = 100.0;  // FIXME arbitrary
	static constexpr double LIFT_CLOSED_POSITION = 0.0;

	pros::Motor m_lift_motor;
	pros::Motor m_clamp_motor;
	bool m_is_open = false;
public:
	Claw(uint8_t const lift_port, uint8_t const clamp_port) : m_lift_motor{ lift_port }, m_clamp_motor{ clamp_port } {}
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

	void set_lift_angle(double const angle, int32_t const velocity = LIFT_VELOCITY) {
		m_lift_motor.move_absolute(std::clamp(angle, LIFT_CLOSED_POSITION, LIFT_OPEN_POSITION), velocity);
	}
	void raise_lift() {
		set_lift_angle(LIFT_OPEN_POSITION);
	}
	void lower_lift() {
		set_lift_angle(LIFT_CLOSED_POSITION);
	}
	void stop_lift() {
		m_lift_motor.move(0);
	}
protected:
	void reify_open_state() {
		m_clamp_motor.move_absolute(m_is_open ? CLAMP_OPEN_POSITION : CLAMP_CLOSED_POSITION, CLAMP_VELOCITY);
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
		m_front_right.set_value(-forward_axis + strafe_axis + rotate_axis);
		m_back_left.set_value(forward_axis - strafe_axis + rotate_axis);
		m_back_right.set_value(-forward_axis - strafe_axis + rotate_axis);
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
	Claw claw{ 1, 2 };  // FIXME arbitrary ports
public:
	Robot() {}

	void update() {
		update_claw();
		update_drivetrain();
	}
protected:
	void update_claw() {
		update_claw_clamp();
		update_claw_arm();
	}
	void update_claw_clamp() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			claw.set_open(true);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			claw.set_open(false);
		}
	}
	void update_claw_arm() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			claw.raise_lift();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			claw.lower_lift();
		} else {
			claw.stop_lift();
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
