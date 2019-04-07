/*
 * MIT License
 *
 * Copyright (c) 2019 FRC Team 980 ThunderBots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.team980.robot2019;

/**
 * Robot-wide parameters are stored here.
 */
@SuppressWarnings("WeakerAccess")
public final class Parameters {

    // JOYSTICKS
    public static final int DRIVE_STICK_ID = 0;
    public static final int DRIVE_WHEEL_ID = 1;
    public static final int XBOX_CONTROLLER_ID = 2;
    public static final int OPERATOR_BOX_ID = 3;

    // AUTONOMOUS
    public static final double AUTO_MIN_SPEED = 0.75; //in ft/sec
    public static final double AUTO_MAX_SPEED = 4.0; //in ft/sec

    public static final double AUTO_MOVE_DIVISOR = 0.5;
    public static final double AUTO_TURN_CORRECTION_DIVISOR = 15;

    public static final double AUTO_TURN_DIVISOR = 100.0;
    public static final double AUTO_TURN_THRESHOLD = 0.4; //in degrees

    public static final double AUTO_FRONT_TRACKING_SPEED = 2.0; //in ft/sec
    public static final double AUTO_BACK_TRACKING_SPEED = -2.0; //in ft/sec
    public static final double AUTO_LOADING_STATION_TARGET_SCORING_WIDTH = 200;
    public static final double AUTO_ROCKET_TARGET_SCORING_WIDTH = 200;

    public static final double AUTO_VISION_CORRECTION_DIVISOR = 120;

    // DRIVE SYSTEM: SPEED CONTROLLERS
    public static final int LEFT_TOP_DRIVE_CONTROLLER_CAN_ID = 0;
    public static final int LEFT_FRONT_DRIVE_CONTROLLER_CAN_ID = 1;
    public static final int LEFT_BACK_DRIVE_CONTROLLER_CAN_ID = 2;
    public static final int RIGHT_TOP_DRIVE_CONTROLLER_CAN_ID = 3;
    public static final int RIGHT_FRONT_DRIVE_CONTROLLER_CAN_ID = 4;
    public static final int RIGHT_BACK_DRIVE_CONTROLLER_CAN_ID = 5;

    // DRIVE SYSTEM: ENCODERS
    public static final int LEFT_DRIVE_ENCODER_CHANNEL_A = 2;
    public static final int LEFT_DRIVE_ENCODER_CHANNEL_B = 3;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_CHANNEL_A = 4;
    public static final int RIGHT_DRIVE_ENCODER_CHANNEL_B = 5;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;

    public static final double DRIVE_ENCODER_PULSES_PER_REVOLUTION = 2048.0;

    // DRIVE SYSTEM: VELOCITY CONTROL
    public static final double MAX_DRIVE_SPEED = 17.0; //in ft/sec

    public static final double LOW_GEAR_PROPORTIONAL_COEFFICIENT = 0.1;
    public static final double LOW_GEAR_INTEGRAL_COEFFICIENT = 0.0;
    public static final double LOW_GEAR_DERIVATIVE_COEFFICIENT = 0.0;
    public static final double LOW_GEAR_FEEDFORWARD_TERM = 0.0;

    public static final double HIGH_GEAR_PROPORTIONAL_COEFFICIENT = 0.03;
    public static final double HIGH_GEAR_INTEGRAL_COEFFICIENT = 0.0;
    public static final double HIGH_GEAR_DERIVATIVE_COEFFICIENT = 0.0;
    public static final double HIGH_GEAR_FEEDFORWARD_TERM = 0.0;

    public static final double DRIVE_PID_TOLERANCE = 0.01;

    // DRIVE SYSTEM: INPUT MODIFIERS
    public static final double DRIVE_STICK_DEADBAND = 0.1;
    public static final double DRIVE_WHEEL_DEADBAND = 0.05;

    public static final double DRIVE_WHEEL_COEFFICIENT = 0.75;

    // DRIVE SYSTEM: SHIFTERS
    public static final int SHIFTER_SOLENOID_PCM_CHANNEL = 0;

    public static final double DRIVE_SHIFT_UP_POINT = 4.5; // in ft/sec
    public static final double DRIVE_SHIFT_DOWN_POINT = 4.0; // in ft/sec

    // ROBOT ARM: SPEED CONTROLLERS
    public static final int SHOULDER_CONTROLLER_CAN_ID = 11;
    public static final int ELBOW_CONTROLLER_CAN_ID = 12;
    public static final int WRIST_CONTROLLER_CAN_ID = 13;

    // ROBOT ARM: MOTOR CONTROL
    public static final double ARM_ENCODER_DEADBAND = 1.0;

    public static final double MIN_SHOULDER_ANGLE = 70;
    public static final double MAX_SHOULDER_ANGLE = 210;

    public static final double MIN_SHOULDER_SPEED = 1.0; //in degrees/sec
    public static final double MAX_SHOULDER_SPEED = 30.0; //in degrees/sec

    public static final double SHOULDER_VELOCITY_DIVISOR = 180.0;
    public static final double SHOULDER_PROPORTIONAL_COEFFICIENT = 0.00025;

    public static final boolean INVERT_SHOULDER_MOTOR = true;

    public static final double MIN_ELBOW_ANGLE = 25;
    public static final double MAX_ELBOW_ANGLE = 140;

    public static final double MIN_ELBOW_SPEED = 25.0; //in degrees/sec
    public static final double MAX_ELBOW_SPEED = 60.0; //in degrees/sec

    public static final double ELBOW_VELOCITY_DIVISOR = 180.0;
    public static final double ELBOW_PROPORTIONAL_COEFFICIENT = 0.00025;

    public static final boolean INVERT_ELBOW_MOTOR = true;

    public static final double MIN_WRIST_ANGLE = 80;
    public static final double MAX_WRIST_ANGLE = 320;

    public static final double MIN_WRIST_SPEED = 20.0; //in degrees/sec
    public static final double MAX_WRIST_SPEED = 60.0; //in degrees/sec

    public static final double WRIST_VELOCITY_DIVISOR = 115.0;
    public static final double WRIST_PROPORTIONAL_COEFFICIENT = 0.00025;

    public static final boolean INVERT_WRIST_MOTOR = false;

    public static final double FINE_WRIST_CONTROL_COEFFICIENT = 0.25;

    // END EFFECTOR
    public static final int INTAKE_CONTROLLER_CAN_ID = 15;
    public static final int HATCH_GRABBER_SOLENOID_PCM_CHANNEL = 1;

    public static final int INTAKE_DIRECTION_INVERT = 1;

    public static final double INTAKE_CONTROLLER_DEADBAND = 0.1;

    // "CLIMB SYSTEM" (lol)
    public static final int SKID_PLATE_DEPLOY_PCM_CHANNEL = 2;

    // SENSORS
    public static final int IMU_CAN_ID = 0;

    // CONSTANTS
    public static final double TAU = 2 * Math.PI;
    public static final double WHEEL_RADIUS = 2.0; // in inches
}
