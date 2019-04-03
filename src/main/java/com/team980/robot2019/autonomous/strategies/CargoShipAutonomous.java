package com.team980.robot2019.autonomous.strategies;

import com.team980.robot2019.autonomous.Autonomous;
import com.team980.robot2019.autonomous.subcommands.EncoderMove;
import com.team980.robot2019.autonomous.subcommands.IMUTurn;
import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.EndEffector;
import com.team980.robot2019.subsystems.RobotArm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class CargoShipAutonomous extends CommandGroup {

    public CargoShipAutonomous(DriveSystem driveSystem, RobotArm robotArm, EndEffector endEffector, double[] ypr, Rioduino rioduino, Autonomous.Side side) {
        super("CargoShipAutonomous");

        // -- Jump off of hab --

        // Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 11.0, 1.0));

        // Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // Instantaneous hard brake
        // Shift into low gear
        addSequential(new InstantCommand(() -> {
            //driveSystem.setSetpoints(0, 0);
            driveSystem.setGear(DriveSystem.Gear.LOW);
        }));

        // Disable arm security and return to stowed
        addSequential(new InstantCommand(() -> {
            robotArm.setSecurityEnabled(false); //TODO do I need this?
            robotArm.setPose(RobotArm.Pose.CARGO_SHIP_CARGO);
        }));

        // -- Score first cargo --

        // Turn to angle
        addSequential(new IMUTurn(driveSystem, ypr, -10 * side.invert));

        // Drive forward to first cargo bay
        addSequential(new EncoderMove(driveSystem, ypr, 10.5));

        // Turn to face cargo bay
        addSequential(new IMUTurn(driveSystem, ypr, 90 * side.invert));

        // Drive forward to cargo ship
        addSequential(new EncoderMove(driveSystem, ypr, 1.5));

        // Score!
        //addSequential(new InstantCommand(() -> endEffector.setIntake(EndEffector.IntakeDirection.OUT, 0.5)));

        // Wait a second
        //addSequential(new TimedMove(driveSystem, ypr, 0.0, 1.0)); //Stop in place

        // Shut off intakes
        //addSequential(new InstantCommand(() -> endEffector.setIntake(EndEffector.IntakeDirection.STOPPED, 0.0)));
    }
}
