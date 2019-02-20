package com.team980.robot2019.autonomous;

import com.team980.robot2019.autonomous.subcommands.*;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

import static com.team980.robot2019.Parameters.AUTO_FRONT_TRACKING_SPEED;
import static com.team980.robot2019.Parameters.AUTO_ROCKET_TARGET_SCORING_WIDTH;

public final class Autonomous extends CommandGroup {

    private Autonomous(DriveSystem driveSystem, double[] ypr, Rioduino rioduino, Side side) {
        super("Autonomous"); //TODO logging solution for Commands

        // 1. Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 10.0, 1.0));

        // 2. Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // 3. Instantaneous hard brake
        addSequential(new InstantCommand(() -> driveSystem.setSetpoints(0, 0)));

        // 4. Shift into low gear
        addSequential(new InstantCommand(() -> driveSystem.setGear(DriveSystem.Gear.LOW)));

        // NOTE: The following section is DEPRECATED and should be updated at competition

        // 5. Turn to overshot angle
        addSequential(new IMUTurn(driveSystem, ypr, -54.5 * side.invert));

        // 6. Drive forward to midpoint
        addSequential(new EncoderMove(driveSystem, ypr, 7.0));

        // 7: Turn to face rocket
        addSequential(new IMUTurn(driveSystem, ypr, -30 * side.invert));

        // 8. Use Pixy to drive to target (and score)
        addSequential(new VisionTrack(driveSystem, rioduino,
                AUTO_FRONT_TRACKING_SPEED, AUTO_ROCKET_TARGET_SCORING_WIDTH));

        //TODO port rest of auto :D
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
