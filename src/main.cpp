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

class MotorGroup {
protected:
	std::vector<pros::Motor> m_motor_group;
	const int32_t m_velocity;
	const double m_open_position;

	double saved_position;

	static constexpr double CLOSED_POSITION = 0.0;
	static constexpr double CLOSED_THRESHOLD = 20.0;
public:
	MotorGroup(std::vector<pros::Motor> const motor_group, int32_t const velocity, double const open_position)
		: m_motor_group{ motor_group }, m_velocity{ velocity }, m_open_position{ open_position } {}

	void set_position(double angle, int32_t const velocity) {
		angle = std::clamp(angle, CLOSED_POSITION, m_open_position);
		for (pros::Motor& m : m_motor_group) {
			m.move_absolute(angle, velocity);
		}
	}
	double position() const {
		return m_motor_group[0].get_position();
	}

	void raise() {
		set_position(m_open_position, m_velocity);
		saved_position = position();
	}

	void lower() {
		set_position(CLOSED_POSITION, m_velocity);
		saved_position = position();
	}

	void stay() {
		if (saved_position > CLOSED_THRESHOLD) {
			set_position(saved_position, MOTOR_MAX_VOLTAGE);
		}
	}
};

class Drivetrain {
protected:
	pros::Motor m_front_left, m_front_right, m_back_left, m_back_right;
	static constexpr double DAMPENING = 1.0 / 2.0;
public:
	Drivetrain(uint8_t const front_left_port, uint8_t const front_right_port, uint8_t const back_left_port, uint8_t const back_right_port)
		: m_front_left{ front_left_port }, m_front_right{ front_right_port, true }, m_back_left{ back_left_port }, m_back_right{ back_right_port, true } {}

	void update(double const _forward_axis, double const _strafe_axis, double const _rotate_axis) {
		auto const forward_axis = transform_motor_power(_forward_axis);
		auto const strafe_axis = transform_motor_power(_strafe_axis);
		auto const rotate_axis = transform_motor_power(_rotate_axis);

		m_front_left.move(forward_axis + strafe_axis + rotate_axis);
		m_back_left.move(forward_axis - strafe_axis + rotate_axis);
		// right motors are inverted
		m_front_right.move(forward_axis - strafe_axis - rotate_axis);
		m_back_right.move(forward_axis + strafe_axis - rotate_axis);
	}
protected:
	static int32_t transform_motor_power(double const power_out_of_one) {
		return static_cast<int32_t>(power_out_of_one * power_out_of_one * power_out_of_one * MOTOR_MAX_VOLTAGE * DAMPENING);
	}
};

class Robot {
protected:
	pros::Controller controller{ pros::E_CONTROLLER_MASTER };
	Drivetrain drivetrain{ 16, 15, 7, 8 };
	MotorGroup tray{ std::vector{ pros::Motor{ 3 } }, 20, 800.0 };  // FIXME arbitrary ports
	MotorGroup arm{ std::vector{ pros::Motor{ 9 }, pros::Motor{ 10, true } }, 35, 1000.0 };
public:
	Robot() {}

	void update() {
		update_drivetrain();
		update_arm();
		update_tray();
	}
protected:
	void update_tray() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			tray.raise();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			tray.lower();
		} else {
			tray.stay();
		}
	}
	void update_arm() {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			arm.raise();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			arm.lower();
		} else {
			arm.stay();
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

static const pros::controller_digital_e_t BUTTONS[] = {
	pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT,
	pros::E_CONTROLLER_DIGITAL_X,  pros::E_CONTROLLER_DIGITAL_B,    pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A,
};

static pros::controller_digital_e_t next_button() {
	while (true) {
		for (unsigned long i = 0; i < sizeof(BUTTONS) / sizeof(BUTTONS[0]); ++i) {
			if (pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, BUTTONS[i])) {
				return BUTTONS[i];
			}
		}
	}
}

static bool authenticate() {
	uint32_t seed = 123;
	while (true) {
		auto const button = next_button();
		if (button == pros::E_CONTROLLER_DIGITAL_R2) {
			break;
		} else {
			seed ^= static_cast<uint32_t>(button) + 0x9e37779b9 + (seed << 6) + (seed >> 2);
		}
	}
	return seed == 4248382663;
}

void opcontrol() {
	if (!authenticate()) {
		return;
	}

	Robot robot;

	while (true) {
		robot.update();
		pros::Task::delay(1);  // sched_yield
	}
}
