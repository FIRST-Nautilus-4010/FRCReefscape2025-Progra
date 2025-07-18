package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Manages the motor controllers for the elevator subsystem,
 * including configuration and basic control methods.
 */
public class ElevatorIO {
    private final TalonFX krakenLBottom = new TalonFX(16);
    private final TalonFX krakenLTop = new TalonFX(15);
    private final TalonFX krakenRBottom = new TalonFX(17);
    private final TalonFX krakenRTop = new TalonFX(18);

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public ElevatorIO() {
        krakenLTop.setControl(new Follower(krakenLBottom.getDeviceID(), false));
        krakenRTop.setControl(new Follower(krakenRBottom.getDeviceID(), false));
    }

    /**
     * @return The left leader motor.
     */
    public TalonFX getLeftMotor() {
        return krakenLBottom;
    }

    /**
     * @return The right leader motor.
     */
    public TalonFX getRightMotor() {
        return krakenRBottom;
    }

    /**
     * Sets the motor power for both leader motors.
     *
     * @param left  Power for the left motor.
     * @param right Power for the right motor.
     */
    public void setMotorPowers(double left, double right) {
        krakenLBottom.set(left);
        krakenRBottom.set(right);
    }
}
