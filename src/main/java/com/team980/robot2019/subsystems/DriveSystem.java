package com.team980.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import static com.team980.robot2019.Parameters.*;

/**
 * Implements velocity control for our West Coast drive system.
 * <p>
 * Note: Much of the non-PID control logic is ported from the {@link DifferentialDrive} class
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

    // Mildly annoying hack
    private WPI_TalonSRX[] motorArray = new WPI_TalonSRX[6];

    public DriveSystem() {
        var leftTopMotor = new WPI_TalonSRX(LEFT_TOP_DRIVE_CONTROLLER_CAN_ID);
        leftTopMotor.setInverted(true);
        var leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT_DRIVE_CONTROLLER_CAN_ID);
        var leftBackMotor = new WPI_TalonSRX(LEFT_BACK_DRIVE_CONTROLLER_CAN_ID);

        motorArray[0] = leftTopMotor;
        motorArray[1] = leftFrontMotor;
        motorArray[2] = leftBackMotor;

        leftDrive = new SpeedControllerGroup(leftTopMotor, leftFrontMotor, leftBackMotor);
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
        var rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT_DRIVE_CONTROLLER_CAN_ID);
        var rightBackMotor = new WPI_TalonSRX(RIGHT_BACK_DRIVE_CONTROLLER_CAN_ID);

        motorArray[3] = rightTopMotor;
        motorArray[4] = rightFrontMotor;
        motorArray[5] = rightBackMotor;

        rightDrive = new SpeedControllerGroup(rightTopMotor, rightFrontMotor, rightBackMotor);
        rightDrive.setInverted(true);
        rightDrive.setName("Drive System", "Right Speed Controllers");

        rightEncoder = new Encoder(RIGHT_DRIVE_ENCODER_CHANNEL_A, RIGHT_DRIVE_ENCODER_CHANNEL_B, INVERT_RIGHT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        rightEncoder.setDistancePerPulse((TAU * (WHEEL_RADIUS / 12)) / DRIVE_ENCODER_PULSES_PER_REVOLUTION);
        rightEncoder.setPIDSourceType(PIDSourceType.kRate);
        //TODO make sure I don't need to reverse this - just to be safe
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

    public void setMotorSafetyEnabled(boolean isMotorSafetyEnabled) {
        for (WPI_TalonSRX motor : motorArray) {
            motor.setSafetyEnabled(isMotorSafetyEnabled);
        }
    }

    /**
     * Separated so it can run in all control modes
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
     * @param left         Requested left drive command, from -1 to 1
     * @param right        Requested right drive command, from -1 to 1
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    public void tankDrive(double left, double right, boolean squareInputs) {
        //left = limit(left);
        left = applyDeadband(left, DRIVE_STICK_DEADBAND);

        //right = limit(right);
        right = applyDeadband(right, DRIVE_STICK_DEADBAND);

        if (squareInputs) {
            left = Math.copySign(left * left, left);
            right = Math.copySign(right * right, right);
        }

        if (isPIDEnabled()) {
            leftController.setSetpoint(left * MAX_DRIVE_SPEED);
            rightController.setSetpoint(right * MAX_DRIVE_SPEED);
        } else {
            leftDrive.set(left);
            rightDrive.set(right);
        }

        runAutoShift();
    }

    /**
     * @param move         Requested move command, from -1 to 1
     * @param turn         Requested turn command, from -1 to 1
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    public void arcadeDrive(double move, double turn, boolean squareInputs) {
        //move = limit(move);
        move = applyDeadband(move, DRIVE_STICK_DEADBAND);

        //turn = limit(turn);
        turn = applyDeadband(turn, DRIVE_WHEEL_DEADBAND);

        if (squareInputs) {
            move = Math.copySign(move * move, move);
            turn = Math.copySign(turn * turn, turn);
        }

        double left;
        double right;

        var maxInput = Math.copySign(Math.max(Math.abs(move), Math.abs(turn)), move);

        if (move >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
                left = maxInput;
                right = move - turn;
            } else {
                left = move + turn;
                right = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
                left = move + turn;
                right = maxInput;
            } else {
                left = maxInput;
                right = move - turn;
            }
        }

        if (isPIDEnabled()) {
            leftController.setSetpoint(left * MAX_DRIVE_SPEED);
            rightController.setSetpoint(right * MAX_DRIVE_SPEED);
        } else {
            leftDrive.set(left);
            rightDrive.set(right);
        }

        runAutoShift();
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    private double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public void disable() {
        setSetpoints(0, 0);
        setPIDEnabled(false);

        leftController.reset();
        rightController.reset();

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
