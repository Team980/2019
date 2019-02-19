package com.team980.robot2019.autonomous;

import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class Autonomous extends CommandGroup {

    private Autonomous(DriveSystem driveSystem, double[] ypr, Side side) {
        super("Autonomous"); //TODO logging solution for Commands

        // 1. Drive forward (1s) until on slope of platform
        addSequential(new TimedMove(driveSystem, ypr, 10.0, 1.0));

        // 2. Drive forward until IMU stabilizes
        addSequential(new TiltAwareMove(driveSystem, ypr, 5.0));

        // 3. Hard brake for 0.25 seconds
        addSequential(new TimedMove(driveSystem, ypr, 0, 1.0));

        // 4. Shift into low gear
        addSequential(new InstantCommand(() -> driveSystem.setGear(DriveSystem.Gear.LOW)));

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

        public Builder(DriveSystem driveSystem, double[] ypr) {
            this.driveSystem = driveSystem;
            this.ypr = ypr;
        }

        public Autonomous build(Side side) {
            return new Autonomous(driveSystem, ypr, side);
        }
    }
}
