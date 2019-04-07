package com.team980.robot2019.autonomous;

import com.team980.robot2019.autonomous.strategies.CargoShipAutonomous;
import com.team980.robot2019.autonomous.strategies.CargoShipPlusFetchAutonomous;
import com.team980.robot2019.autonomous.strategies.LaunchOnlyAutonomous;
import com.team980.robot2019.autonomous.strategies.RocketHatchAutonomous;
import com.team980.robot2019.sensors.Rioduino;
import com.team980.robot2019.subsystems.DriveSystem;
import com.team980.robot2019.subsystems.EndEffector;
import com.team980.robot2019.subsystems.RobotArm;
import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class Autonomous {

    public enum Side {
        RIGHT(1),
        LEFT(-1);

        public double invert;

        Side(double invert) {
            this.invert = invert;
        }
    }

    public enum Strategy {
        TWO_HATCH,
        LAUNCH_ONLY,
        CARGO_SHIP,
        CARGO_SHIP_PLUS_FETCH
    }

    public static final class Builder {

        private DriveSystem driveSystem;
        private RobotArm robotArm;
        private EndEffector endEffector;
        private double[] ypr;
        private Rioduino rioduino;

        public Builder(DriveSystem driveSystem, RobotArm robotArm, EndEffector endEffector, double[] ypr, Rioduino rioduino) {
            this.driveSystem = driveSystem;
            this.robotArm = robotArm;
            this.endEffector = endEffector;
            this.ypr = ypr;
            this.rioduino = rioduino;
        }

        public CommandGroup build(Side side, Strategy strategy) {
            switch (strategy) {
                case TWO_HATCH:
                    return new RocketHatchAutonomous(driveSystem, robotArm, ypr, rioduino, side);
                case LAUNCH_ONLY:
                    return new LaunchOnlyAutonomous(driveSystem, ypr);
                case CARGO_SHIP:
                    return new CargoShipAutonomous(driveSystem, robotArm, endEffector, ypr, rioduino, side);
                case CARGO_SHIP_PLUS_FETCH:
                    return new CargoShipPlusFetchAutonomous(driveSystem, robotArm, endEffector, ypr, rioduino, side);
                default:
                    return build(side, Strategy.CARGO_SHIP); //Catch null and return default
            }
        }
    }
}
