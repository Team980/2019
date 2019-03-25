package com.team980.robot2019.autonomous.subcommands;

import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static com.team980.robot2019.Parameters.*;

public final class EncoderMove extends Command {

    private DriveSystem driveSystem;
    private double[] ypr;

    private double initialHeading;

    private final double distance;

    private boolean isFinished = false;

    public EncoderMove(DriveSystem driveSystem, double[] ypr, double distance) {
        this.driveSystem = driveSystem;
        this.ypr = ypr;

        this.distance = distance;
    }

    @Override
    protected void initialize() {
        Shuffleboard.addEventMarker("EncoderMove: " + distance + " feet", EventImportance.kTrivial);
        System.out.println("EncoderMove: " + distance + "feet");

        driveSystem.resetEncoders();

        initialHeading = ypr[0];
    }

    @Override
    protected void execute() {
        if (Math.abs(driveSystem.getLeftEncoder().getDistance()) < Math.abs(distance)
                && Math.abs(driveSystem.getRightEncoder().getDistance()) < Math.abs(distance)) {

            var differential = distance;
            if (Math.abs(driveSystem.getLeftEncoder().getDistance()) > Math.abs(driveSystem.getRightEncoder().getDistance())) {
                differential -= driveSystem.getLeftEncoder().getDistance();
            } else {
                differential -= driveSystem.getRightEncoder().getDistance();
            }

            var speed = (differential / AUTO_MOVE_DIVISOR) * AUTO_MAX_SPEED;

            if (Math.abs(speed) < AUTO_MIN_SPEED) speed = Math.copySign(AUTO_MIN_SPEED, speed);
            if (Math.abs(speed) > AUTO_MAX_SPEED) speed = Math.copySign(AUTO_MAX_SPEED, speed);

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
