package com.team980.robot2019.autonomous;

import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.RobotArm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class TwoHatchAutonomous extends CommandGroup implements Autonomous {

    TwoHatchAutonomous(DriveSystem driveSystem, RobotArm robotArm, double[] ypr, Rioduino rioduino, Autonomous.Side side) {
        super("TwoHatchAutonomous");

        // 1. Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 11.0, 1.0));

        // 2. Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // 3. Instantaneous hard brake
        // 4. Shift into low gear
        addSequential(new InstantCommand(() -> {
            driveSystem.setSetpoints(0, 0);
            driveSystem.setGear(DriveSystem.Gear.LOW);
        }));

        // 4b. Disable arm security and return to stowed
        addSequential(new InstantCommand(() -> {
            robotArm.setSecurityEnabled(false);
            robotArm.setPose(RobotArm.Pose.STOWED); //TODO MID_HATCH?
        }));


        // 5. Turn to overshot angle
        //addSequential(new IMUTurn(driveSystem, ypr, -54.5 * side.invert));

        // 6. Drive forward to midpoint
        //addSequential(new EncoderMove(driveSystem, ypr, 7.0));

        // 7: Turn to face rocket
        //addSequential(new IMUTurn(driveSystem, ypr, -30 * side.invert));

        // 8. Use Pixy to drive to target (and score)
        //addSequential(new VisionTrack(driveSystem, rioduino, AUTO_FRONT_TRACKING_SPEED, AUTO_ROCKET_TARGET_SCORING_WIDTH));

        // 9. Back up short length
        //addSequential(new EncoderMove(driveSystem, ypr, -2.5));

        // 10. Turn so we face away from loading station
        //addSequential(new IMUTurn(driveSystem, ypr, 180 * side.invert));

        // 11. Drive to loading station
        //addSequential(new EncoderMove(driveSystem, ypr, 10.0));

        // 12. Use Pixy to pick up from loading station
        //addSequential(new VisionTrack(driveSystem, rioduino, AUTO_FRONT_TRACKING_SPEED, AUTO_LOADING_STATION_TARGET_SCORING_WIDTH));

        // 13. Inch backward
        //addSequential(new EncoderMove(driveSystem, ypr, -3.0));

        // 14. Turn to slight angle
        //addSequential(new IMUTurn(driveSystem, ypr, 187.5 * side.invert));

        // 15. Drive to center of field
        //addSequential(new EncoderMove(driveSystem, ypr, -18.0));

        // 16. Turn to far side of rocket
        //addSequential(new IMUTurn(driveSystem, ypr, 45 * side.invert));

        // 17. Drive towards rocket
        //addSequential(new EncoderMove(driveSystem, ypr, -3.0));

        // 18. Turn directly to rocket
        //addSequential(new IMUTurn(driveSystem, ypr, 30 * side.invert));

        // 19. Use Pixy to score
        //addSequential(new VisionTrack(driveSystem, cameraProcessor, AUTO_BACK_TRACKING_SPEED, AUTO_ROCKET_TARGET_SCORING_WIDTH));
    }
}
