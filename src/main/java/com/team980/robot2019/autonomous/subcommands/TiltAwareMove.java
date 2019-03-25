package com.team980.robot2019.autonomous.subcommands;

import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static com.team980.robot2019.Parameters.*;

public final class TiltAwareMove extends Command {

    private DriveSystem driveSystem;
    private double[] ypr;

    private double initialHeading;

    private final double pitchThreshold;

    private boolean isFinished = false;

    public TiltAwareMove(DriveSystem driveSystem, double[] ypr, double pitchThreshold) {
        this.driveSystem = driveSystem;
        this.ypr = ypr;

        this.pitchThreshold = pitchThreshold;
    }

    @Override
    protected void initialize() {
        Shuffleboard.addEventMarker("TiltAwareMove", EventImportance.kTrivial);
        System.out.println("TiltAwareMove");

        driveSystem.resetEncoders();

        initialHeading = ypr[0];
    }

    @Override
    protected void execute() {
        if (Math.abs(ypr[1]) > pitchThreshold) { //Drive until we are on a level surface

            var speed = AUTO_MAX_SPEED;
            var turnCorrect = (initialHeading - ypr[0]) / AUTO_TURN_CORRECTION_DIVISOR;

            driveSystem.setSetpoints(speed - turnCorrect, speed + turnCorrect);
        } else {
            isFinished = true;
        }
    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    @Override
    protected void end() {
        driveSystem.setSetpoints(0, 0);
    }
}
