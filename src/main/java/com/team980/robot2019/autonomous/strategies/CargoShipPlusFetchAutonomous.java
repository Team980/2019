package com.team980.robot2019.autonomous.strategies;

import com.team980.robot2019.autonomous.Autonomous;
import com.team980.robot2019.autonomous.subcommands.EncoderMove;
import com.team980.robot2019.autonomous.subcommands.IMUTurn;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.EndEffector;
import com.team980.robot2019.subsystems.RobotArm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class CargoShipPlusFetchAutonomous extends CommandGroup {

    public CargoShipPlusFetchAutonomous(DriveSystem driveSystem, RobotArm robotArm, EndEffector endEffector, double[] ypr, Rioduino rioduino, Autonomous.Side side) {
        super("CargoShipPlusFetchAutonomous");

        // -- Inherit from Cargo Ship autonomous --
        addSequential(new CargoShipAutonomous(driveSystem, robotArm, endEffector, ypr, rioduino, side));

        // -- Navigate to loading station -- TODO test

        // Back up away from cargo bay
        addSequential(new EncoderMove(driveSystem, ypr, -1.0));

        // Turn towards hab
        addSequential(new IMUTurn(driveSystem, ypr, 210.0 * side.invert));

        // Move arm to grab hatch
        addSequential(new InstantCommand(() -> {
            robotArm.setSecurityEnabled(true);
            robotArm.setPose(RobotArm.Pose.LOW_ROCKET_HATCH);
        }));

        // Move to line up with loading station
        //addSequential(new EncoderMove(driveSystem, ypr, 17.0));

        // Turn to face loading station
        //addSequential(new IMUTurn(driveSystem, ypr, 180.0 * side.invert));

        // Use Pixy to reach loading station
        //addSequential(new VisionTrack(driveSystem, rioduino, AUTO_FRONT_TRACKING_SPEED, AUTO_LOADING_STATION_TARGET_SCORING_WIDTH));
    }
}
