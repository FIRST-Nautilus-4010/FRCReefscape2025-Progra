package frc.robot.subsystems.elevators;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Manages the motor controllers for the elevator subsystem,
 * including configuration and basic control methods.
 */
public class ElevatorIO {
    private final TalonFX krakenRR;
    private final TalonFX krakenRL;
    private final TalonFX krakenLR;
    private final TalonFX krakenLL;

    /**
     * Configures follower motors to mirror their respective leaders.
     */
    public ElevatorIO() {
        krakenRR = new TalonFX(ElevatorConstants.KRAKEN_RR_ID);
        krakenRL = new TalonFX(ElevatorConstants.KRAKEN_RL_ID);
        krakenLR = new TalonFX(ElevatorConstants.KRAKEN_LR_ID);
        krakenLL = new TalonFX(ElevatorConstants.KRAKEN_LL_ID);

        krakenRL.setControl(new Follower(krakenRR.getDeviceID(), false));
        krakenLL.setControl(new Follower(krakenRR.getDeviceID(), true));
        krakenLR.setControl(new Follower(krakenRR.getDeviceID(), true));
    }

    /**
     * @return The leader motor.
     */
    public TalonFX getLeaderMotor() {
        return krakenRR;
    }

    /**
     * 
     */
    public void setVoltage(double voltage) {
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

    public double getVelocity() {
        return krakenRR.getVelocity().getValue().magnitude();
    }
}