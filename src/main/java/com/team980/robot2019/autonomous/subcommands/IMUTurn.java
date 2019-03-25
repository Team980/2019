package com.team980.robot2019.autonomous.subcommands;

import com.team980.robot2019.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static com.team980.robot2019.Parameters.*;

public final class IMUTurn extends Command {

    private DriveSystem driveSystem;
    private double[] ypr; //data from Pigeon IMU

    private final double heading;

    private boolean isFinished = false;

    public IMUTurn(DriveSystem driveSystem, double[] ypr, double heading) {
        this.driveSystem = driveSystem;
        this.ypr = ypr;

        this.heading = heading;
    }

    @Override
    protected void initialize() {
        Shuffleboard.addEventMarker("IMUTurn: " + heading + " degrees", EventImportance.kTrivial);
        System.out.println("IMUTurn: " + heading + " degrees");
    }

    @Override
    protected void execute() {
        if (Math.abs(heading - ypr[0]) > AUTO_TURN_THRESHOLD) {

            var differential = heading - ypr[0];

            var speed = (differential / AUTO_TURN_DIVISOR) * AUTO_MAX_SPEED;

            if (Math.abs(speed) < AUTO_MIN_SPEED) speed = Math.copySign(AUTO_MIN_SPEED, speed);
            if (Math.abs(speed) > AUTO_MAX_SPEED) speed = Math.copySign(AUTO_MAX_SPEED, speed);

            driveSystem.setSetpoints(-speed, speed);
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
