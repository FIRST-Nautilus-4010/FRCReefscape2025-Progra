package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIO {
    private final TalonFX motorPosition;
    private final TalonFX motorVelocity;

    public IntakeIO(){
        motorPosition = new TalonFX(IntakeConstants.motorPosition);
        motorVelocity = new TalonFX(IntakeConstants.motorVelocity);
    }

    public void setPosition(double pose){
        motorPosition.set(pose);
    }

    public void setVelocity(double vel){
        motorVelocity.set(vel);
    }

    public double getVel() {
        return motorVelocity.getVelocity().getValueAsDouble();
    }

    public double getPos() {
        return motorPosition.getPosition().getValueAsDouble();
    }

    public TalonFX getMotorPosition() {
        return motorPosition;
    }

    public TalonFX getMotorVelocity() {
        return motorVelocity;
    }

    public double getRollersAmp() {
        return motorVelocity.getStatorCurrent().getValueAsDouble();
    }
}
