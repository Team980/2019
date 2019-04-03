package com.team980.robot2019.autonomous.strategies;

import com.team980.robot2019.autonomous.Autonomous;
import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.RobotArm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

@Deprecated
public final class RocketHatchAutonomous extends CommandGroup {

    public RocketHatchAutonomous(DriveSystem driveSystem, RobotArm robotArm, double[] ypr, Rioduino rioduino, Autonomous.Side side) {
        super("RocketHatchAutonomous");

        // -- Jump off of hab --

        // Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 11.0, 1.0));

        // Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // Instantaneous hard brake
        // Shift into low gear
        addSequential(new InstantCommand(() -> {
            driveSystem.setSetpoints(0, 0);
            driveSystem.setGear(DriveSystem.Gear.LOW);
        }));

        // Disable arm security and return to stowed
        addSequential(new InstantCommand(() -> {
            robotArm.setSecurityEnabled(false);
            robotArm.setPose(RobotArm.Pose.STOWED);
        }));

        // -- Score hatch on rocket (TODO)

        // Turn to overshot angle
        //addSequential(new IMUTurn(driveSystem, ypr, -54.5 * side.invert));

        // Drive forward to midpoint
        //addSequential(new EncoderMove(driveSystem, ypr, 7.0));

        // Turn to face rocket
        //addSequential(new IMUTurn(driveSystem, ypr, -30 * side.invert));

        // Use Pixy to drive to target (and score)
        //addSequential(new VisionTrack(driveSystem, rioduino, AUTO_FRONT_TRACKING_SPEED, AUTO_ROCKET_TARGET_SCORING_WIDTH));

        // Back up short length
        //addSequential(new EncoderMove(driveSystem, ypr, -2.5));

        // Turn so we face away from loading station
        //addSequential(new IMUTurn(driveSystem, ypr, 180 * side.invert));

        // Drive to loading station
        //addSequential(new EncoderMove(driveSystem, ypr, 10.0));

        // Use Pixy to pick up from loading station
        //addSequential(new VisionTrack(driveSystem, rioduino, AUTO_FRONT_TRACKING_SPEED, AUTO_LOADING_STATION_TARGET_SCORING_WIDTH));

        // Inch backward
        //addSequential(new EncoderMove(driveSystem, ypr, -3.0));

        // Turn to slight angle
        //addSequential(new IMUTurn(driveSystem, ypr, 187.5 * side.invert));

        // Drive to center of field
        //addSequential(new EncoderMove(driveSystem, ypr, -18.0));

        // Turn to far side of rocket
        //addSequential(new IMUTurn(driveSystem, ypr, 45 * side.invert));

        // Drive towards rocket
        //addSequential(new EncoderMove(driveSystem, ypr, -3.0));

        // Turn directly to rocket
        //addSequential(new IMUTurn(driveSystem, ypr, 30 * side.invert));

        // Use Pixy to score
        //addSequential(new VisionTrack(driveSystem, cameraProcessor, AUTO_BACK_TRACKING_SPEED, AUTO_ROCKET_TARGET_SCORING_WIDTH));
    }
}
