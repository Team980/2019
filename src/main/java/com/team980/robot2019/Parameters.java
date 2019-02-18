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

    // DRIVE SYSTEM: SPEED CONTROLLERS
    public static final int LEFT_TOP_DRIVE_CONTROLLER_CAN_ID = 0;
    public static final int LEFT_FRONT_DRIVE_CONTROLLER_CAN_ID = 1;
    public static final int LEFT_BACK_DRIVE_CONTROLLER_CAN_ID = 2;
    public static final int RIGHT_TOP_DRIVE_CONTROLLER_CAN_ID = 3;
    public static final int RIGHT_FRONT_DRIVE_CONTROLLER_CAN_ID = 4;
    public static final int RIGHT_BACK_DRIVE_CONTROLLER_CAN_ID = 5;

    // DRIVE SYSTEM: ENCODERS TODO hook up
    public static final int LEFT_DRIVE_ENCODER_CHANNEL_A = 2;
    public static final int LEFT_DRIVE_ENCODER_CHANNEL_B = 3;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_CHANNEL_A = 4;
    public static final int RIGHT_DRIVE_ENCODER_CHANNEL_B = 5;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;

    public static final double DRIVE_ENCODER_PULSES_PER_REVOLUTION = 2048.0;

    // DRIVE SYSTEM: VELOCITY CONTROL TODO tune PID
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
    public static final double DRIVE_STICK_DEADBAND = 0.05;
    public static final double DRIVE_STICK_SHIFT_POINT = 0.5;

    public static final double LOW_GEAR_DRIVE_STICK_COEFFICIENT = 8.5;
    public static final double HIGH_GEAR_DRIVE_STICK_COEFFICIENT = 25.5;
    public static final double HIGH_GEAR_DRIVE_STICK_OFFSET = 8.5;

    public static final double DRIVE_WHEEL_DEADBAND = 0.1;
    public static final double DRIVE_WHEEL_COEFFICIENT = 0.5;

    // DRIVE SYSTEM: SHIFTERS
    public static final int SHIFTER_SOLENOID_PCM_CHANNEL = 0;

    public static final double DRIVE_SHIFT_UP_POINT = 4.5; // in ft/sec
    public static final double DRIVE_SHIFT_DOWN_POINT = 4.0; // in ft/sec

    // CONSTANTS
    public static final double TAU = 2 * Math.PI;
    public static final double WHEEL_RADIUS = 2.0; // in inches
}
