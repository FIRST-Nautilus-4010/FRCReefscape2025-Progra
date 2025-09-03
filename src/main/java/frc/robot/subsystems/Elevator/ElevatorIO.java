package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Manages the motor controllers for the elevator subsystem,
 * including configuration and basic control methods.
 */
public class ElevatorIO {
    private final TalonFX krakenRR = new TalonFX(ElevatorConstants.KRAKEN_RR_ID);
    private final TalonFX krakenRL = new TalonFX(ElevatorConstants.KRAKEN_RL_ID);
    private final TalonFX krakenLR = new TalonFX(ElevatorConstants.KRAKEN_LR_ID);
    private final TalonFX krakenLL = new TalonFX(ElevatorConstants.KRAKEN_LL_ID);

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public ElevatorIO() {
        krakenRL.setControl(new Follower(krakenRR.getDeviceID(), false));
        krakenLL.setControl(new Follower(krakenRR.getDeviceID(), true));
        krakenLR.setControl(new Follower(krakenRR.getDeviceID(), true));
    }

    /**
     * @return The left leader motor.
     */
    public TalonFX getLeaderMotor() {
        return krakenRR;
    }

    /**
     * Sets the motor power for both leader motors.
     *
     * @param left  Power for the left motor.
     * @param right Power for the right motor.
     */
    public void setPower(double voltage) {
        krakenRR.setControl(new VoltageOut(voltage));
    }

    /**
     * Stops both motors immediately.
     */
    public void stop() {
        krakenRR.stopMotor();
    }

    /**
     * Calculates elevator height by measuring the differential in motor rotation.
     *
     * @return Height in meters.
     */
    public double getHeight() {
        return krakenRR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;
    
    }
}