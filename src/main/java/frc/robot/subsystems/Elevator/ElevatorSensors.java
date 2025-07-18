package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorSensors {
    private final DutyCycleEncoder angleEncoder;
    private final TalonFX krakenL, krakenR;

    public ElevatorSensors(TalonFX krakenL, TalonFX krakenR) {
        this.krakenL = krakenL;
        this.krakenR = krakenR;
        this.angleEncoder = new DutyCycleEncoder(2);
    }

    public double getHeight() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        return Math.abs(right - left);
    }

    public double getAngle() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        return Math.abs(right + left);
    }
}
