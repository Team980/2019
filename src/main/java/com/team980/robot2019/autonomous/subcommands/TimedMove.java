package com.team980.robot2019.autonomous.subcommands;

import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static com.team980.robot2019.Parameters.*;

public final class TimedMove extends Command {

    private DriveSystem driveSystem;
    private double[] ypr;

    private double initialHeading;

    private final double speed;
    private final double period;

    private Timer timer;

    private boolean isFinished = false;

    public TimedMove(DriveSystem driveSystem, double[] ypr, double speed, double period) {
        this.driveSystem = driveSystem;
        this.ypr = ypr;

        this.speed = speed;
        this.period = period;

        timer = new Timer();
    }

    @Override
    protected void initialize() {
        Shuffleboard.addEventMarker("TimedMove: " + period + " seconds", EventImportance.kTrivial);

        driveSystem.resetEncoders();

        initialHeading = ypr[0];

        timer.reset();
        timer.start();
    }

    @Override
    protected void execute() {
        if (!timer.hasPeriodPassed(period)) {

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
