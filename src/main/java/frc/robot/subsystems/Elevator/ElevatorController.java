package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialDutyCycle;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;

public class ElevatorController {
    private final TalonFX leftLeader;
    private final TalonFX rightLeader;
    private final SimpleDifferentialMechanism differential;

    public ElevatorController(TalonFX leftLeader, TalonFX rightLeader) {
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;

        TalonFXConfiguration config = createBaseConfiguration();

        configureSlot0(config.Slot0);
        configureSlot1(config.Slot1);
        configureMotionMagic(config.MotionMagic);

        applyConfigurationToMotors(config);

        this.differential = new SimpleDifferentialMechanism(leftLeader, rightLeader, true);
    }

    private TalonFXConfiguration createBaseConfiguration() {
        return new TalonFXConfiguration();
    }

    private void configureSlot0(Slot0Configs slot0) {
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 4.8;
        slot0.kI = 0.0;
        slot0.kD = 0.1;
    }

    private void configureSlot1(Slot1Configs slot1) {
        slot1.kS = 0.25;
        slot1.kV = 0.12;
        slot1.kA = 0.01;
        slot1.kP = 4.8;
        slot1.kI = 0.0;
        slot1.kD = 0.1;
    }

    private void configureMotionMagic(MotionMagicConfigs mm) {
        mm.MotionMagicCruiseVelocity = 80;   // rot/s
        mm.MotionMagicAcceleration = 160;    // rot/s²
        mm.MotionMagicJerk = 1600;           // rot/s³
    }

    private void applyConfigurationToMotors(TalonFXConfiguration config) {
        leftLeader.getConfigurator().apply(config);
        rightLeader.getConfigurator().apply(config);
    }

    public void setPower( double heightPwr, double anglePwr) {
        differential.setControl(new DifferentialDutyCycle(anglePwr, heightPwr));
    }

    public void setTargetPosition(double height, double angle) {
        differential.setControl(new DifferentialMotionMagicVoltage(angle, height));
    }

    public double getAngle() {
        double left = leftLeader.getPosition().getValueAsDouble();
        double right = rightLeader.getPosition().getValueAsDouble();

        return (right + left) / 2.0;
    }

    public double getHeight() {
        double left = leftLeader.getPosition().getValueAsDouble();
        double right = rightLeader.getPosition().getValueAsDouble();

        return (right - left) / 2.0;
    }

    public void periodic() {
        differential.periodic();
    }
}
