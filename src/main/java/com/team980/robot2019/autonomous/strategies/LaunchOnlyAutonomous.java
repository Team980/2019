package com.team980.robot2019.autonomous.strategies;

import com.team980.robot2019.autonomous.subcommands.TiltAwareMove;
import com.team980.robot2019.autonomous.subcommands.TimedMove;
import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class LaunchOnlyAutonomous extends CommandGroup {

    public LaunchOnlyAutonomous(DriveSystem driveSystem, double[] ypr) {
        super("LaunchOnlyAutonomous");

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
    }
}
