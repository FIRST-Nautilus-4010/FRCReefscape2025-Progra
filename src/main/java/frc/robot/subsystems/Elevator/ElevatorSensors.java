package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Handles sensor readings for the elevator mechanism, including height and angle.
 */
public class ElevatorSensors {
    private final DutyCycleEncoder angleEncoder;
    private final TalonFX krakenL, krakenR;

    /**
     * Initializes the sensors with references to the left and right motors.
     */
    public ElevatorSensors(TalonFX krakenL, TalonFX krakenR) {
        this.krakenL = krakenL;
        this.krakenR = krakenR;
        this.angleEncoder = new DutyCycleEncoder(2);
    }

    /**
     * Calculates elevator height by measuring the differential in motor rotation.
     *
     * @return Height in meters.
     */
    public double getHeight() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        return Math.abs(right - left);
    }

    /**
     * Calculates elevator angle from combined motor rotation.
     *
     * @return Angle in radians.
     */
    public double getAngle() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        return right + left;
    }
}
