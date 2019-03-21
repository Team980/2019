package com.team980.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team980.robot2019.sensors.Rioduino;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static com.team980.robot2019.Parameters.*;

public class RobotArm {

    /**
     * Used to access absolute encoder readings
     */
    private Rioduino rioduino;

    //private WPI_TalonSRX shoulderMotor;
    private WPI_TalonSRX elbowMotor;
    private WPI_TalonSRX wristMotor;

    //private State shoulderState;
    private State elbowState;
    private State wristState;

    private Pose pose;

    private int[] stictionBuffer;
    private double[] voltageBuffer;

    public RobotArm(Rioduino rioduino) {
        this.rioduino = rioduino;

        /*shoulderMotor = new WPI_TalonSRX(SHOULDER_CONTROLLER_CAN_ID);
        shoulderMotor.setInverted(INVERT_SHOULDER_MOTOR);
        shoulderMotor.setName("Robot Arm", "Shoulder Speed Controller");*/

        elbowMotor = new WPI_TalonSRX(ELBOW_CONTROLLER_CAN_ID);
        elbowMotor.setInverted(INVERT_ELBOW_MOTOR);
        elbowMotor.setName("Robot Arm", "Elbow Speed Controller");

        wristMotor = new WPI_TalonSRX(WRIST_CONTROLLER_CAN_ID);
        wristMotor.setInverted(INVERT_WRIST_MOTOR);
        wristMotor.setName("Robot Arm", "Wrist Speed Controller");

        stictionBuffer = new int[]{0, 0};
        voltageBuffer = new double[]{0, 0, 0};
    }

    public void initialize() {
        //shoulderState = State.STOPPED;
        elbowState = State.STOPPED;
        wristState = State.STOPPED;

        pose = Pose.STOWED;
    }

    public void setPose(Pose pose) {
        stictionBuffer = new int[]{0, 0}; //TODO configure stiction buffer
        voltageBuffer = new double[]{0, 0, 0};

        this.pose = pose;

        //shoulderState = State.MOVING_TO_POSITION;
        elbowState = State.MOVING_TO_POSITION;
        wristState = State.MOVING_TO_POSITION;
    }

    public void manualControl(double shoulderCmd, double elbowCmd, double wristCmd) {
        /*if (Math.abs(shoulderCmd) > 0.2) {
            shoulderMotor.set(shoulderCmd);
        } else {
            shoulderMotor.set(0);
        }*/

        if (Math.abs(elbowCmd) > 0.2) {
            elbowMotor.set(elbowCmd);
        } else {
            elbowMotor.set(0);
        }

        if (Math.abs(wristCmd) > 0.2) {
            wristMotor.set(wristCmd);
        } else {
            wristMotor.set(0);
        }
    }

    public void automatedControl() {

        // Use consistent angles for the entire loop
        var shoulderAngle = rioduino.getShoulderAngle();
        var elbowAngle = rioduino.getElbowAngle();
        var wristAngle = rioduino.getWristAngle();

        // Fetch angular joint velocities
        var shoulderVelocity = rioduino.getShoulderVelocity();
        var elbowVelocity = rioduino.getElbowVelocity();
        var wristVelocity = rioduino.getWristVelocity();

        // Deactivate automated control if we exit our bounds
        if (/*shoulderAngle < MIN_SHOULDER_ANGLE || shoulderAngle > MAX_SHOULDER_ANGLE
                ||*/ elbowAngle < MIN_ELBOW_ANGLE || elbowAngle > MAX_ELBOW_ANGLE
                || wristAngle < MIN_WRIST_ANGLE || wristAngle > MAX_WRIST_ANGLE) {
            //shoulderState = State.STOPPED;
            elbowState = State.STOPPED;
            wristState = State.STOPPED;
            Shuffleboard.addEventMarker("Arm exited bounds, disabling automation", EventImportance.kCritical);

        }

        // Individual joint control
        /*if (shoulderState == State.MOVING_TO_POSITION) {
            if (stictionBuffer[0] > 0) {
                var positionDelta = pose.shoulderAngle - shoulderAngle;

                shoulderMotor.set(Math.copySign(1.0, positionDelta));

                stictionBuffer[2]--;
            } else {
                var positionDelta = pose.shoulderAngle - shoulderAngle;
                var desiredVelocity = (positionDelta / SHOULDER_VELOCITY_DIVISOR) * MAX_SHOULDER_SPEED;

                if (Math.abs(desiredVelocity) < MIN_SHOULDER_SPEED) desiredVelocity = Math.copySign(MIN_SHOULDER_SPEED, desiredVelocity);
                if (Math.abs(desiredVelocity) > MAX_SHOULDER_SPEED) desiredVelocity = Math.copySign(MAX_SHOULDER_SPEED, desiredVelocity);

                var velocityDelta = desiredVelocity - shoulderVelocity;
                voltageBuffer[2] += (velocityDelta * SHOULDER_PROPORTIONAL_COEFFICIENT);

                if (Math.abs(voltageBuffer[2]) > 1.0) voltageBuffer[2] = Math.copySign(1.0, voltageBuffer[2]);

                if (Math.abs(positionDelta) > ARM_ENCODER_DEADBAND) {
                    shoulderMotor.set(voltageBuffer[2]);
                } else {
                    shoulderMotor.set(0);
                    shoulderState = State.STOPPED;
                }
            }
        } else {
            shoulderMotor.set(0);
        }*/

        if (elbowState == State.MOVING_TO_POSITION) {
            if (stictionBuffer[1] > 0) {
                var positionDelta = pose.elbowAngle - elbowAngle;

                elbowMotor.set(Math.copySign(1.0, positionDelta));

                stictionBuffer[2]--;
            } else {
                var positionDelta = pose.elbowAngle - elbowAngle;
                var desiredVelocity = (positionDelta / ELBOW_VELOCITY_DIVISOR) * MAX_ELBOW_SPEED;

                if (Math.abs(desiredVelocity) < MIN_ELBOW_SPEED) desiredVelocity = Math.copySign(MIN_ELBOW_SPEED, desiredVelocity);
                if (Math.abs(desiredVelocity) > MAX_ELBOW_SPEED) desiredVelocity = Math.copySign(MAX_ELBOW_SPEED, desiredVelocity);

                var velocityDelta = desiredVelocity - elbowVelocity;
                voltageBuffer[2] += (velocityDelta * ELBOW_PROPORTIONAL_COEFFICIENT);

                if (Math.abs(voltageBuffer[2]) > 1.0) voltageBuffer[2] = Math.copySign(1.0, voltageBuffer[2]);

                if (Math.abs(positionDelta) > ARM_ENCODER_DEADBAND) {
                    elbowMotor.set(voltageBuffer[2]);
                } else {
                    elbowMotor.set(0);
                    elbowState = State.STOPPED;
                }
            }
        } else {
            elbowMotor.set(0);
        }

        if (wristState == State.MOVING_TO_POSITION) {
            var positionDelta = pose.wristAngle - wristAngle;
            var desiredVelocity = (positionDelta / WRIST_VELOCITY_DIVISOR) * MAX_WRIST_SPEED;

            if (Math.abs(desiredVelocity) < MIN_WRIST_SPEED) desiredVelocity = Math.copySign(MIN_WRIST_SPEED, desiredVelocity);
            if (Math.abs(desiredVelocity) > MAX_WRIST_SPEED) desiredVelocity = Math.copySign(MAX_WRIST_SPEED, desiredVelocity);

            var velocityDelta = desiredVelocity - wristVelocity;
            voltageBuffer[2] += (velocityDelta * WRIST_PROPORTIONAL_COEFFICIENT);

            if (Math.abs(voltageBuffer[2]) > 1.0) voltageBuffer[2] = Math.copySign(1.0, voltageBuffer[2]);

            if (Math.abs(positionDelta) > ARM_ENCODER_DEADBAND) {
                wristMotor.set(voltageBuffer[2]);
            } else {
                wristMotor.set(0);
                wristState = State.STOPPED;
            }
        } else {
            wristMotor.set(0);
        }
    }

    public void disable() {
        //shoulderState = State.STOPPED;
        elbowState = State.STOPPED;
        wristState = State.STOPPED;

        //shoulderMotor.stopMotor();
        elbowMotor.stopMotor();
        wristMotor.stopMotor();
    }

    public enum State {
        MOVING_TO_POSITION,
        STOPPED
    }

    public enum Pose { //TODO calculate all positions
        STOWED(345, 80, 180),

        MID_ROCKET_HATCH(345, 96, 242),
        MID_ROCKET_CARGO(345, 97, 186),

        LOW_ROCKET_HATCH(345, 155, 302),
        LOW_ROCKET_CARGO(345, 130, 185),

        FLOOR_HATCH_PICKUP(345, 167, 223),
        FLOOR_CARGO_PICKUP(345, 144, 126);

        double shoulderAngle;
        double elbowAngle;
        double wristAngle;

        Pose(double shoulderAngle, double elbowAngle, double wristAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
            this.wristAngle = wristAngle;
        }
    }
}
