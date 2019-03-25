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

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team980.robot2019.autonomous.Autonomous;
import com.team980.robot2019.autonomous.CargoShipAutonomous;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.EndEffector;
import com.team980.robot2019.subsystems.RobotArm;
import com.team980.robot2019.util.TeleopScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import static com.team980.robot2019.Parameters.*;

/**
 * Base robot class for FRC Robot programming.
 * Periodic methods are called on a 20ms timer.
 */
public final class Robot extends TimedRobot {

    private NetworkTable dataTable;

    private Joystick driveStick;
    private Joystick driveWheel;
    private XboxController xboxController;
    private Joystick operatorBox;

    private PigeonIMU imu;
    private double[] ypr; //Stores yaw/pitch/roll from IMU

    private Rioduino rioduino;

    private DriveSystem driveSystem;
    private RobotArm robotArm;
    private EndEffector endEffector;

    private Solenoid skidPlateDeploySolenoid;

    private CargoShipAutonomous.Builder autonomous;
    private SendableChooser<Autonomous.Side> sideChooser; //TODO choose Class as well!!

    private TeleopScheduler teleopScheduler;

    private boolean tipProtectionEnabled = false;
    private boolean autoTargetEnabled = false;

    /**
     * Robot-wide initialization code goes here.
     * Called ONCE when the robot is powered on.
     */
    @Override
    public void robotInit() {
        dataTable = NetworkTableInstance.getDefault().getTable("Data"); //Custom data table for read-only sensor data

        driveStick = new Joystick(DRIVE_STICK_ID);
        driveWheel = new Joystick(DRIVE_WHEEL_ID);
        xboxController = new XboxController(XBOX_CONTROLLER_ID);
        operatorBox = new Joystick(OPERATOR_BOX_ID);

        imu = new PigeonIMU(IMU_CAN_ID);
        ypr = new double[3];

        rioduino = new Rioduino();

        driveSystem = new DriveSystem();
        robotArm = new RobotArm(rioduino);
        endEffector = new EndEffector();

        skidPlateDeploySolenoid = new Solenoid(SKID_PLATE_DEPLOY_PCM_CHANNEL);
        skidPlateDeploySolenoid.setName("Skid Plate Deploy Solenoid");

        autonomous = new Autonomous.Builder(driveSystem, robotArm, endEffector, ypr, rioduino);

        sideChooser = new SendableChooser<>();
        sideChooser.addOption("Left", Autonomous.Side.LEFT);
        sideChooser.addOption("Right", Autonomous.Side.RIGHT);
        sideChooser.setName("Autonomous", "Side Selection");
        LiveWindow.add(sideChooser);

        teleopScheduler = new TeleopScheduler();
    }

    /**
     * Runs periodically in all robot modes.
     */
    @Override
    public void robotPeriodic() {
        imu.getYawPitchRoll(ypr);

        rioduino.updateData();

        // Push updated sensor data to data table
        dataTable.getSubTable("IMU").getEntry("Yaw").setNumber(ypr[0]);
        dataTable.getSubTable("IMU").getEntry("Pitch").setNumber(ypr[1]);
        dataTable.getSubTable("IMU").getEntry("Roll").setNumber(ypr[2]);

        dataTable.getSubTable("Vision").getSubTable("Front Camera").getEntry("Target Center Coord").setNumber(rioduino.getTargetCenterCoord());
        dataTable.getSubTable("Vision").getSubTable("Front Camera").getEntry("Target Width").setNumber(rioduino.getTargetWidth());

        if (sideChooser.getSelected() != null) {
            dataTable.getSubTable("Autonomous").getEntry("Side").setString(sideChooser.getSelected().name());
        } else {
            dataTable.getSubTable("Autonomous").getEntry("Side").setString("NULL");
        }

        dataTable.getSubTable("Status Flags").getEntry("Tip Protection").setBoolean(tipProtectionEnabled);
        dataTable.getSubTable("Status Flags").getEntry("Auto Target").setBoolean(autoTargetEnabled);

        robotArm.updateData(dataTable);
    }

    /**
     * Called once at the beginning of the autonomous period.
     */
    @Override
    public void autonomousInit() {
        Shuffleboard.setRecordingFileNameFormat("LAR_AUTO_${time}");
        Shuffleboard.startRecording();

        imu.setYaw(0, 0);

        driveSystem.setGear(DriveSystem.Gear.HIGH);
        driveSystem.setPIDEnabled(true);
        driveSystem.setAutoShiftEnabled(false);

        driveSystem.resetEncoders();

        robotArm.setSecurityEnabled(true);

        robotArm.initialize();

        endEffector.setHatchGrabberExtended(false);

        Autonomous.Side side;
        if (sideChooser.getSelected() != null) {
            side = sideChooser.getSelected();
        } else {
            side = Autonomous.Side.RIGHT;
        }
        autonomous.buildCargoShip(Autonomous.Side.RIGHT).start(); //TODO don't hardcode these choices!
    }

    /**
     * Runs periodically in the autonomous period.
     */
    @Override
    public void autonomousPeriodic() {
        if (driveStick.getRawButton(7)) { // Auto kill switch - numbered button on base
            Scheduler.getInstance().disable();
            Scheduler.getInstance().removeAll();

            driveSystem.disable();
            robotArm.disable();
            endEffector.disable();
        }

        Scheduler.getInstance().run();

        robotArm.automatedControl();
    }

    /**
     * Called once at the beginning of the teleoperated period.
     */
    @Override
    public void teleopInit() {
        Shuffleboard.setRecordingFileNameFormat("LAR_TELEOP_${time}");
        Shuffleboard.startRecording();

        driveSystem.resetEncoders();

        driveSystem.setGear(DriveSystem.Gear.LOW);
        driveSystem.setPIDEnabled(true);
        driveSystem.setAutoShiftEnabled(true);

        robotArm.setSecurityEnabled(true);

        endEffector.setIntake(EndEffector.IntakeDirection.STOPPED, 0.0);

        teleopScheduler.initialize();

        // Rumble controller at start of teleop
        xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 1);

        teleopScheduler.queue(0.5, () -> {
            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        });

        // Rumble controller at T=15 before end of match
        teleopScheduler.queue(105, () -> {
            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
        });

        teleopScheduler.queue(107, () -> {
            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        });
    }

    /**
     * Runs periodically in the teleoperated period.
     */
    @Override
    public void teleopPeriodic() {

        // Auto shifting
        if (operatorBox.getRawButton(2) || operatorBox.getRawButton(3)) { //Manual shifting lock - tri-state switch
            if (operatorBox.getRawButton(2) && driveSystem.getGear() == DriveSystem.Gear.LOW) {
                driveSystem.setAutoShiftEnabled(false);
                driveSystem.setGear(DriveSystem.Gear.HIGH);
            } else if (operatorBox.getRawButton(3) && driveSystem.getGear() == DriveSystem.Gear.HIGH) {
                driveSystem.setAutoShiftEnabled(false);
                driveSystem.setGear(DriveSystem.Gear.LOW);
            }
        } else {
            driveSystem.setAutoShiftEnabled(true);
        }

        // Robot drive - handles all override moves
        if (tipProtectionEnabled) { // Tipping protection overrides driver input
            autoTargetEnabled = false;
            driveSystem.arcadeDrive(Math.copySign(0.4, ypr[1]), 0);
        } else if (autoTargetEnabled) {
            var targetCenterOffset = rioduino.getTargetCenterCoord() - 160 - 25; // Normalize coordinates, account for off center
            var turnSpeed = targetCenterOffset / AUTO_VISION_CORRECTION_DIVISOR;

            if (rioduino.getTargetCenterCoord() == -1) {
                autoTargetEnabled = false;
                turnSpeed = 0; // No targets detected
            }

            if (Math.abs(targetCenterOffset) < 2.0) {
                autoTargetEnabled = false;
                turnSpeed = 0; // Target locked
            }

            driveSystem.setSetpoints(turnSpeed, -turnSpeed);
        } else {
            driveSystem.arcadeDrive(-driveStick.getY(), driveWheel.getX());
        }

        // Tip protection activation
        if (driveWheel.getRawButton(5)) { //Driver override for tipping protection - left wheel paddle
            tipProtectionEnabled = false;
        } else {
            if (Math.abs(ypr[1]) >= 10) { //Activate tipping protection
                tipProtectionEnabled = true;
                Shuffleboard.addEventMarker("Tip protection enabled", EventImportance.kHigh);
                System.out.println("Tip protection enabled");

            } else if (Math.abs(ypr[1]) < 1 && tipProtectionEnabled) { //Disable when safe
                tipProtectionEnabled = false;
                Shuffleboard.addEventMarker("Tip protection disabled", EventImportance.kHigh);
                System.out.println("Tip protection disabled");
            }
        }

        // Arm control
        if (operatorBox.getRawButton(1)) { //Manual arm control - big red switch
            robotArm.manualControl(0, xboxController.getY(GenericHID.Hand.kLeft),
                    xboxController.getY(GenericHID.Hand.kRight));
        } else {
            robotArm.fineWristControl(xboxController.getY(GenericHID.Hand.kLeft) * 0.25); //TODO store as constant?
            robotArm.automatedControl();
        }

        // Arm poses
        if (xboxController.getStickButtonPressed(GenericHID.Hand.kRight)) {
            robotArm.setPose(RobotArm.Pose.STOWED_CARGO_PRELOAD); //STOWED

        } else if (xboxController.getAButtonPressed()) {
            robotArm.setPose(RobotArm.Pose.LOW_ROCKET_HATCH);

        } else if (xboxController.getBButtonPressed()) {
            robotArm.setPose(RobotArm.Pose.MID_ROCKET_HATCH);

        } else if (xboxController.getXButtonPressed()) {
            robotArm.setPose(RobotArm.Pose.FLOOR_HATCH_PICKUP);

        } else if (xboxController.getPOV() == 180) { //Down on d-pad
            robotArm.setPose(RobotArm.Pose.LOW_ROCKET_CARGO);

        } else if (xboxController.getPOV() == 90) { //Right on d-pad
            robotArm.setPose(RobotArm.Pose.MID_ROCKET_CARGO);

        /*} else if (xboxController.getPOV() == 270) { //Left on d-pad
            robotArm.setPose(RobotArm.Pose.FLOOR_CARGO_PICKUP);*/

        } else if (xboxController.getPOV() == 0) { //Up on d-pad
            robotArm.setPose(RobotArm.Pose.CARGO_SHIP_CARGO);

        } else if ((xboxController.getPOV() == 270)) { // TODO move when elbow is fixed
            robotArm.setPose(RobotArm.Pose.LOADING_STATION_CARGO);

        }

        // Cargo intake
        if (xboxController.getTriggerAxis(GenericHID.Hand.kRight) > INTAKE_CONTROLLER_DEADBAND) {
            endEffector.setIntake(EndEffector.IntakeDirection.IN, xboxController.getTriggerAxis(GenericHID.Hand.kRight));
        } else if (xboxController.getTriggerAxis(GenericHID.Hand.kLeft) > INTAKE_CONTROLLER_DEADBAND) {
            endEffector.setIntake(EndEffector.IntakeDirection.OUT, xboxController.getTriggerAxis(GenericHID.Hand.kLeft));
        } else {
            endEffector.setIntake(EndEffector.IntakeDirection.STOPPED, 0);
        }

        // Hatch intake
        if (xboxController.getBumperPressed(GenericHID.Hand.kRight)) {
            endEffector.setHatchGrabberExtended(true);
        } else if (xboxController.getBumperPressed(GenericHID.Hand.kLeft)) {
            endEffector.setHatchGrabberExtended(false);
        }

        // Skid plate deployment
        if (driveStick.getRawButtonPressed(2)) {
            skidPlateDeploySolenoid.set(!skidPlateDeploySolenoid.get());
        }

        teleopScheduler.execute();
    }

    /**
     * Called once when the robot is disabled.
     * NOTE that this will be called in between the autonomous and teleoperated periods!
     */
    @Override
    public void disabledInit() {
        driveSystem.disable();
        robotArm.disable();
        endEffector.disable();

        xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);

        Scheduler.getInstance().removeAll();
        teleopScheduler.disable();

        Shuffleboard.stopRecording();
        Shuffleboard.clearRecordingFileNameFormat();
    }
}