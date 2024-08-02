// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
// #include <string>
// #include <deque>

#include <frc/TimedRobot.h>
// #include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <ctre/phoenix6/TalonFX.hpp>
// #include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include "ctre/Phoenix.h"

#include "LogitechConstants.hpp"
// #include "SenderNT.hpp"

#include <wpi/sendable/SendableBuilder.h>

// #include "wpimath/MathShared.h"


using namespace ctre::phoenix6;

using TalonFX6 = ctre::phoenix6::hardware::TalonFX;
using SimTalonFX6 = ctre::phoenix6::sim::TalonFXSimState;

static constexpr auto
// motor physical speed targets
	TRENCHER_MAX_VELO = 80_tps,				// maximum mining speed
	TRENCHER_NOMINAL_MINING_VELO = 80_tps,	// base trenching speed
	HOPPER_BELT_MAX_VELO = 45_tps,
	HOPPER_BELT_MAX_MINING_VELO = 10_tps,
	TRACKS_MAX_VELO = 125_tps,
	TRACKS_MINING_VELO = 8_tps,
	TRACKS_MAX_ADDITIONAL_MINING_VEL = 6_tps,
	TRACKS_OFFLOAD_VELO = TRACKS_MAX_VELO * 0.25;

static constexpr auto
	MOTOR_SETPOINT_ACC = 5_tr_per_s_sq;

static constexpr double
// motor constants
	GENERIC_MOTOR_kP = 0.11,	// An error of 1 rotation per second results in 2V output
	GENERIC_MOTOR_kI = 0.5,		// An error of 1 rotation per second increases output by 0.5V every second
	GENERIC_MOTOR_kD = 0.0001,	// A change of 1 rotation per second squared results in 0.0001 volts output
	GENERIC_MOTOR_kV = 0.12,	// Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
// driving
	DRIVING_MAGNITUDE_DEADZONE_SCALAR = 0.1,
	DRIVING_LOW_SPEED_SCALAR = 0.3,
	DRIVING_MEDIUM_SPEED_SCALAR = 0.7,
	DRIVING_HIGH_SPEED_SCALAR = 1.0,
	GENERIC_DEADZONE_SCALAR = 0.05,

static constexpr int
	DISABLE_ALL_ACTIONS_BUTTON_IDX = LogitechConstants::BUTTON_A,

	TELEOP_LOW_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_B,
	TELEOP_MEDIUM_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_Y,
	TELEOP_HIGH_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_X,

	TELEOP_DRIVE_X_AXIS_IDX = LogitechConstants::LEFT_JOY_X,
	TELEOP_DRIVE_Y_AXIS_IDX = LogitechConstants::LEFT_JOY_Y;
