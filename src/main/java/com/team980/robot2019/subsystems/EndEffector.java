package com.team980.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import static com.team980.robot2019.Parameters.*;


public class EndEffector {

    private WPI_TalonSRX intakeMotor;

    private Solenoid hatchGrabberSolenoid;

    public EndEffector() {
        intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_CAN_ID);
        intakeMotor.setName("End Effector", "Intake Motor");

        hatchGrabberSolenoid = new Solenoid(HATCH_GRABBER_SOLENOID_PCM_CHANNEL);
        hatchGrabberSolenoid.setName("End Effector", "Hatch Grabber Solenoid");
    }

    public void setIntake(IntakeDirection direction, double speed) {
        intakeMotor.set(Math.abs(speed) * direction.invert);
    }

    public void setHatchGrabberExtended(boolean isHatchGrabberExtended) {
        hatchGrabberSolenoid.set(isHatchGrabberExtended);
    }

    public void disable() {
        intakeMotor.stopMotor();

        hatchGrabberSolenoid.set(false);
    }

    public enum IntakeDirection {
        IN(INTAKE_DIRECTION_INVERT),
        OUT(-INTAKE_DIRECTION_INVERT),
        STOPPED(0);

        private int invert;

        IntakeDirection(int invert) {
            this.invert = invert;
        }
    }
}
