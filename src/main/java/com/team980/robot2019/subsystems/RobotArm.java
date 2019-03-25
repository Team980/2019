package com.team980.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team980.robot2019.sensors.Rioduino;
import edu.wpi.first.networktables.NetworkTable;
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

    //private State shoulderState = State.STOPPED;
    private State elbowState = State.STOPPED;
    private State wristState = State.STOPPED;

    private Pose pose = Pose.STOWED;

    private int[] stictionBuffer;
    private double[] voltageBuffer;

    private boolean securityEnabled = true;

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

    public void updateData(NetworkTable dataTable) {
        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Position").getEntry("Shoulder").setNumber(rioduino.getShoulderAngle());
        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Position").getEntry("Elbow").setNumber(rioduino.getElbowAngle());
        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Position").getEntry("Wrist").setNumber(rioduino.getWristAngle());

        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Velocity").getEntry("Shoulder").setNumber(rioduino.getShoulderVelocity());
        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Velocity").getEntry("Elbow").setNumber(rioduino.getElbowVelocity());
        dataTable.getSubTable("Robot Arm").getSubTable("Absolute Encoders").getSubTable("Velocity").getEntry("Wrist").setNumber(rioduino.getWristVelocity());

        dataTable.getSubTable("Robot Arm").getEntry("Pose").setString(pose.name());

        //dataTable.getSubTable("Robot Arm").getSubTable("State").getEntry("Shoulder").setString(shoulderState.name());
        dataTable.getSubTable("Robot Arm").getSubTable("State").getEntry("Elbow").setString(elbowState.name());
        dataTable.getSubTable("Robot Arm").getSubTable("State").getEntry("Wrist").setString(wristState.name());
    }

    public boolean isSecurityEnabled() {
        return securityEnabled;
    }

    public void setSecurityEnabled(boolean securityEnabled) {
        this.securityEnabled = securityEnabled;
    }

    public void setPose(Pose pose) {
        stictionBuffer = new int[]{0, 0}; //TODO configure stiction buffer, remove for elbow
        voltageBuffer = new double[]{0, 0, 0};

        this.pose = pose;

        //shoulderState = State.MOVING_TO_POSITION;
        elbowState = State.MOVING_TO_POSITION;
        wristState = State.MOVING_TO_POSITION;

        Shuffleboard.addEventMarker("Pose set: " + pose.name(), EventImportance.kNormal);
        System.out.println("Pose set: " + pose.name());
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

    public void fineWristControl(double wristCmd) {
        if (Math.abs(wristCmd) > 0.05) {
            wristState = State.STOPPED;
            wristMotor.set(wristCmd);
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
        if (securityEnabled) {
            if (/*shoulderAngle < MIN_SHOULDER_ANGLE || shoulderAngle > MAX_SHOULDER_ANGLE
                ||*/ elbowAngle < MIN_ELBOW_ANGLE || elbowAngle > MAX_ELBOW_ANGLE
                    || wristAngle < MIN_WRIST_ANGLE || wristAngle > MAX_WRIST_ANGLE) {
                //shoulderState = State.STOPPED;
                elbowState = State.STOPPED;
                wristState = State.STOPPED;
                Shuffleboard.addEventMarker("Arm exited bounds, disabling automation", EventImportance.kCritical);
                System.out.println("Arm exited bounds, disabling automation");
            }
        }

        // Individual joint control
        /*if (shoulderState == State.MOVING_TO_POSITION) {
            if (stictionBuffer[0] > 0) {
                var positionDelta = pose.shoulderAngle - shoulderAngle;

                shoulderMotor.set(Math.copySign(1.0, positionDelta));

                stictionBuffer[0]--;
            } else {
                var positionDelta = pose.shoulderAngle - shoulderAngle;
                var desiredVelocity = (positionDelta / SHOULDER_VELOCITY_DIVISOR) * MAX_SHOULDER_SPEED;

                if (Math.abs(desiredVelocity) < MIN_SHOULDER_SPEED) desiredVelocity = Math.copySign(MIN_SHOULDER_SPEED, desiredVelocity);
                if (Math.abs(desiredVelocity) > MAX_SHOULDER_SPEED) desiredVelocity = Math.copySign(MAX_SHOULDER_SPEED, desiredVelocity);

                var velocityDelta = desiredVelocity - shoulderVelocity;
                voltageBuffer[0] += (velocityDelta * SHOULDER_PROPORTIONAL_COEFFICIENT);

                if (Math.abs(voltageBuffer[0]) > 1.0) voltageBuffer[2] = Math.copySign(1.0, voltageBuffer[0]);

                if (Math.abs(positionDelta) > ARM_ENCODER_DEADBAND) {
                    shoulderMotor.set(voltageBuffer[0]);
                } else {
                    shoulderMotor.set(0);
                    shoulderState = State.STOPPED;
                }
            }
        } else {
            shoulderMotor.set(0);
        }*/

        if (elbowState == State.MOVING_TO_POSITION) {
            var positionDelta = pose.elbowAngle - elbowAngle;
            var desiredVelocity = (positionDelta / ELBOW_VELOCITY_DIVISOR) * MAX_ELBOW_SPEED;

            if (Math.abs(desiredVelocity) < MIN_ELBOW_SPEED)
                desiredVelocity = Math.copySign(MIN_ELBOW_SPEED, desiredVelocity);
            if (Math.abs(desiredVelocity) > MAX_ELBOW_SPEED)
                desiredVelocity = Math.copySign(MAX_ELBOW_SPEED, desiredVelocity);

            var velocityDelta = desiredVelocity - elbowVelocity;
            voltageBuffer[1] += (velocityDelta * ELBOW_PROPORTIONAL_COEFFICIENT);

            if (Math.abs(voltageBuffer[1]) > 1.0) voltageBuffer[2] = Math.copySign(1.0, voltageBuffer[1]);

            if (Math.abs(positionDelta) > ARM_ENCODER_DEADBAND) {
                elbowMotor.set(voltageBuffer[1]);
            } else {
                elbowMotor.set(0);
                elbowState = State.STOPPED;
            }
        } else {
            elbowMotor.set(0);
        }

        if (wristState == State.MOVING_TO_POSITION) {
            var positionDelta = pose.wristAngle - wristAngle;
            var desiredVelocity = (positionDelta / WRIST_VELOCITY_DIVISOR) * MAX_WRIST_SPEED;

            if (Math.abs(desiredVelocity) < MIN_WRIST_SPEED)
                desiredVelocity = Math.copySign(MIN_WRIST_SPEED, desiredVelocity);
            if (Math.abs(desiredVelocity) > MAX_WRIST_SPEED)
                desiredVelocity = Math.copySign(MAX_WRIST_SPEED, desiredVelocity);

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

        /*if (wristState == State.MOVING_TO_POSITION) { Old voltage control
            var differential = wristAngle - pose.wristAngle;

            var voltage = (differential / 45) * 0.4;

            if (Math.abs(voltage) < 0.2) voltage = Math.copySign(0.2, voltage);
            if (Math.abs(voltage) < 0.4) voltage = Math.copySign(0.4, voltage);

            if (Math.abs(differential) > ARM_ENCODER_DEADBAND) {
                wristMotor.set(voltage);
            } else {
                wristMotor.set(0);
                wristState = State.STOPPED;
            }
        } else {
            wristMotor.set(0);
        }*/
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

    public enum Pose {
        STOWED(-1, 34, 302), //TODO hatch?
        STOWED_CARGO_PRELOAD(-1, 42, 310),

        MID_ROCKET_HATCH(-1, 48, 224),
        MID_ROCKET_CARGO(-1, 38, 151),

        LOW_ROCKET_HATCH(-1, 113, 290),
        LOW_ROCKET_CARGO(-1, 75, 136),

        FLOOR_HATCH_PICKUP(-1, 127, 217),
        FLOOR_CARGO_PICKUP(-1, 104, 119),

        CARGO_SHIP_CARGO(-1, 38, 85),

        LOADING_STATION_CARGO(-1, 38, 115);

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
