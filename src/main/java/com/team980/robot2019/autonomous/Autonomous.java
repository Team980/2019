package com.team980.robot2019.autonomous;

import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class Autonomous extends CommandGroup {

    private Autonomous(DriveSystem driveSystem, double[] ypr, Rioduino rioduino, Side side) {
        super("Autonomous");

        // 1. Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 10.0, 1.0));

        // 2. Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // 3. Instantaneous hard brake
        addSequential(new InstantCommand(() -> driveSystem.setSetpoints(0, 0)));

        // 4. Shift into low gear
        addSequential(new InstantCommand(() -> driveSystem.setGear(DriveSystem.Gear.LOW)));

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

    public enum Side {
        RIGHT(1),
        LEFT(-1);

        public double invert;

        Side(double invert) {
            this.invert = invert;
        }
    }

    public static final class Builder {

        private DriveSystem driveSystem;
        private double[] ypr;
        private Rioduino rioduino;

        public Builder(DriveSystem driveSystem, double[] ypr, Rioduino rioduino) {
            this.driveSystem = driveSystem;
            this.ypr = ypr;
            this.rioduino = rioduino;
        }

        public Autonomous build(Side side) {
            return new Autonomous(driveSystem, ypr, rioduino, side);
        }
    }
}
