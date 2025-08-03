package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Manages the motor controllers for the elevator subsystem,
 * including configuration and basic control methods.
 */
public class ElevatorIO {
    private final TalonFX krakenRR = new TalonFX(10);
    private final TalonFX krakenRL = new TalonFX(11);
    private final TalonFX krakenLR = new TalonFX(12);
    private final TalonFX krakenLL = new TalonFX(13);
    private final DutyCycleEncoder angleEncoder;

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public ElevatorIO() {
        krakenRL.setControl(new Follower(krakenRR.getDeviceID(), false));
        krakenLL.setControl(new Follower(krakenLR.getDeviceID(), false));

        this.angleEncoder = new DutyCycleEncoder(2);
    }

    /**
     * @return The left leader motor.
     */
    public TalonFX getLeftLeaderMotor() {
        return krakenLR;
    }

    /**
     * @return The right leader motor.
     */
    public TalonFX getRightLeaderMotor() {
        return krakenRR;
    }

    /**
     * Sets the motor power for both leader motors.
     *
     * @param left  Power for the left motor.
     * @param right Power for the right motor.
     */
    public void setMotorPowers(double left, double right) {
        if (left != 0) {
            krakenLR.set(left);
        } else {
            krakenLR.stopMotor();
        }
        if (right != 0) {
            krakenRR.set(right);
        } else {
            krakenRR.stopMotor();
        }
    }

    /**
     * Stops both motors immediately.
     */
    public void stopMotors() {
        krakenLR.stopMotor();
        krakenRR.stopMotor();
    }

    /**
     * Calculates elevator height by measuring the differential in motor rotation.
     *
     * @return Height in meters.
     */
    public double getHeight() {
        double right = krakenRR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        double left = krakenLR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
        return Math.abs(right - left);
    }

    /**
     * Calculates elevator angle from combined motor rotation.
     *
     * @return Angle in radians.
     */
    public double getAngle() {
        double right = krakenRR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        double left = krakenLR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN;
        return right + left;
    }
}