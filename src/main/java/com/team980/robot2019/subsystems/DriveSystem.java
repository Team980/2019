package com.team980.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import static com.team980.robot2019.Parameters.*;

/**
 * Implements velocity control for our West Coast drive system.
 */
public final class DriveSystem {

    private SpeedControllerGroup leftDrive;
    private Encoder leftEncoder;
    private PIDController leftController;

    private SpeedControllerGroup rightDrive;
    private Encoder rightEncoder;
    private PIDController rightController;

    private Solenoid shifterSolenoid;

    private boolean isAutoShiftEnabled = false;

    public DriveSystem() {
        var leftTopMotor = new WPI_TalonSRX(LEFT_TOP_DRIVE_CONTROLLER_CAN_ID);
        leftTopMotor.setInverted(true);
        leftDrive = new SpeedControllerGroup(leftTopMotor, new WPI_TalonSRX(LEFT_FRONT_DRIVE_CONTROLLER_CAN_ID), new WPI_TalonSRX(LEFT_BACK_DRIVE_CONTROLLER_CAN_ID));
        leftDrive.setName("Drive System", "Left Speed Controllers");

        leftEncoder = new Encoder(LEFT_DRIVE_ENCODER_CHANNEL_A, LEFT_DRIVE_ENCODER_CHANNEL_B, INVERT_LEFT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        leftEncoder.setDistancePerPulse((TAU * (WHEEL_RADIUS / 12)) / DRIVE_ENCODER_PULSES_PER_REVOLUTION);
        leftEncoder.setPIDSourceType(PIDSourceType.kRate);
        leftEncoder.setName("Drive System", "Left Encoder");

        leftController = new PIDController(LOW_GEAR_PROPORTIONAL_COEFFICIENT, LOW_GEAR_INTEGRAL_COEFFICIENT, LOW_GEAR_DERIVATIVE_COEFFICIENT, LOW_GEAR_FEEDFORWARD_TERM, leftEncoder, leftDrive);
        leftController.setInputRange(-MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
        leftController.setPercentTolerance(DRIVE_PID_TOLERANCE);
        leftController.setName("Drive System", "Left PID Controller");
        LiveWindow.add(leftController);

        var rightTopMotor = new WPI_TalonSRX(RIGHT_TOP_DRIVE_CONTROLLER_CAN_ID);
        rightTopMotor.setInverted(true);
        rightDrive = new SpeedControllerGroup(rightTopMotor, new WPI_TalonSRX(RIGHT_FRONT_DRIVE_CONTROLLER_CAN_ID), new WPI_TalonSRX(RIGHT_BACK_DRIVE_CONTROLLER_CAN_ID));
        rightDrive.setInverted(true);
        rightDrive.setName("Drive System", "Right Speed Controllers");

        rightEncoder = new Encoder(RIGHT_DRIVE_ENCODER_CHANNEL_A, RIGHT_DRIVE_ENCODER_CHANNEL_B, INVERT_RIGHT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        rightEncoder.setDistancePerPulse((TAU * (WHEEL_RADIUS / 12)) / DRIVE_ENCODER_PULSES_PER_REVOLUTION);
        rightEncoder.setPIDSourceType(PIDSourceType.kRate);
        rightEncoder.setName("Drive System", "Right Encoder");

        rightController = new PIDController(LOW_GEAR_PROPORTIONAL_COEFFICIENT, LOW_GEAR_INTEGRAL_COEFFICIENT, LOW_GEAR_DERIVATIVE_COEFFICIENT, LOW_GEAR_FEEDFORWARD_TERM, rightEncoder, rightDrive);
        rightController.setInputRange(-MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
        rightController.setPercentTolerance(DRIVE_PID_TOLERANCE);
        rightController.setName("Drive System", "Right PID Controller");
        LiveWindow.add(rightController);

        shifterSolenoid = new Solenoid(SHIFTER_SOLENOID_PCM_CHANNEL);
        shifterSolenoid.setName("Drive System", "Shifter Solenoid");
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public Gear getGear() {
        return (shifterSolenoid.get() == Gear.LOW.solenoidValue) ? Gear.LOW : Gear.HIGH;
    }

    public void setGear(Gear gear) {
        shifterSolenoid.set(gear.solenoidValue);
    }

    public boolean isPIDEnabled() {
        return leftController.isEnabled() && rightController.isEnabled();
    }

    public void setPIDEnabled(boolean isPIDEnabled) {
        leftController.setEnabled(isPIDEnabled);
        rightController.setEnabled(isPIDEnabled);
    }

    public boolean isAutoShiftEnabled() {
        return isAutoShiftEnabled;
    }

    public void setAutoShiftEnabled(boolean isAutoShiftEnabled) {
        this.isAutoShiftEnabled = isAutoShiftEnabled;
    }

    /**
     * Separated so it can run in both auto and teleop
     */
    private void runAutoShift() {
        if (isAutoShiftEnabled) {
            if (Math.abs(leftEncoder.getRate()) > DRIVE_SHIFT_UP_POINT && Math.abs(rightEncoder.getRate()) > DRIVE_SHIFT_UP_POINT) {
                setGear(Gear.HIGH);

                leftController.setPID(HIGH_GEAR_PROPORTIONAL_COEFFICIENT, HIGH_GEAR_INTEGRAL_COEFFICIENT, HIGH_GEAR_DERIVATIVE_COEFFICIENT, HIGH_GEAR_FEEDFORWARD_TERM);
                rightController.setPID(HIGH_GEAR_PROPORTIONAL_COEFFICIENT, HIGH_GEAR_INTEGRAL_COEFFICIENT, HIGH_GEAR_DERIVATIVE_COEFFICIENT, HIGH_GEAR_FEEDFORWARD_TERM);
            } else if (Math.abs(leftEncoder.getRate()) < DRIVE_SHIFT_DOWN_POINT && Math.abs(rightEncoder.getRate()) < DRIVE_SHIFT_DOWN_POINT) {
                setGear(Gear.LOW);

                leftController.setPID(LOW_GEAR_PROPORTIONAL_COEFFICIENT, LOW_GEAR_INTEGRAL_COEFFICIENT, LOW_GEAR_DERIVATIVE_COEFFICIENT, LOW_GEAR_FEEDFORWARD_TERM);
                rightController.setPID(LOW_GEAR_PROPORTIONAL_COEFFICIENT, LOW_GEAR_INTEGRAL_COEFFICIENT, LOW_GEAR_DERIVATIVE_COEFFICIENT, LOW_GEAR_FEEDFORWARD_TERM);
            }
        }
    }

    /**
     * @param left  Requested left speed, scaled
     * @param right Requested right speed, scaled
     */
    public void setSetpoints(double left, double right) {
        leftController.setSetpoint(left);
        rightController.setSetpoint(right);

        runAutoShift();
    }

    /**
     * @param left  Requested left command, from -1 to 1
     * @param right Requested right command, from -1 to 1
     */
    public void tankDrive(double left, double right) {
        if (Math.abs(left) < DRIVE_STICK_DEADBAND) left = 0;
        if (Math.abs(right) < DRIVE_STICK_DEADBAND) right = 0;

        if (isPIDEnabled()) {
            if (Math.abs(left) > DRIVE_STICK_SHIFT_POINT) {
                left = (left * HIGH_GEAR_DRIVE_STICK_COEFFICIENT) - HIGH_GEAR_DRIVE_STICK_OFFSET;
            } else {
                left *= LOW_GEAR_DRIVE_STICK_COEFFICIENT;
            }

            if (Math.abs(right) > DRIVE_STICK_SHIFT_POINT) {
                right = (right * HIGH_GEAR_DRIVE_STICK_COEFFICIENT) - HIGH_GEAR_DRIVE_STICK_OFFSET;
            } else {
                right *= LOW_GEAR_DRIVE_STICK_COEFFICIENT;
            }

            setSetpoints(left, right);
        } else {
            leftDrive.set(left);
            rightDrive.set(right);

            runAutoShift();
        }
    }

    /**
     * @param move Requested move command, from -1 to 1
     * @param turn Requested turn command, from -1 to 1
     */
    public void arcadeDrive(double move, double turn) {
        if (Math.abs(move) < DRIVE_STICK_DEADBAND) move = 0;
        if (Math.abs(turn) < DRIVE_WHEEL_DEADBAND) turn = 0;

        turn *= DRIVE_WHEEL_COEFFICIENT;

        tankDrive(move + turn, move - turn);
    }

    public void disable() {
        setPIDEnabled(false);

        leftDrive.stopMotor();
        rightDrive.stopMotor();
    }

    public enum Gear {
        LOW(true),
        HIGH(false);

        private boolean solenoidValue;

        Gear(boolean solenoidValue) {
            this.solenoidValue = solenoidValue;
        }
    }
}
