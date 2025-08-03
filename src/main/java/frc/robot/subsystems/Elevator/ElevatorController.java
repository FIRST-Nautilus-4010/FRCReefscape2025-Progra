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
        slot0.kG = ElevatorConstants.SLOT0_KG;
        slot0.kS = ElevatorConstants.SLOT0_KS;
        slot0.kV = ElevatorConstants.SLOT0_KV;
        slot0.kA = ElevatorConstants.SLOT0_KA;
        slot0.kP = ElevatorConstants.SLOT0_KP;
        slot0.kI = ElevatorConstants.SLOT0_KI;
        slot0.kD = ElevatorConstants.SLOT0_KD;
    }

    private void configureSlot1(Slot1Configs slot1) {
        slot1.kG = ElevatorConstants.SLOT1_KG;
        slot1.kS = ElevatorConstants.SLOT1_KS;
        slot1.kV = ElevatorConstants.SLOT1_KV;
        slot1.kA = ElevatorConstants.SLOT1_KA;
        slot1.kP = ElevatorConstants.SLOT1_KP;
        slot1.kI = ElevatorConstants.SLOT1_KI;
        slot1.kD = ElevatorConstants.SLOT1_KD;
    }

    private void configureMotionMagic(MotionMagicConfigs mm) {
        mm.MotionMagicCruiseVelocity = ElevatorConstants.MAGIC_MOTION_VELOCITY;   // rot/s
        mm.MotionMagicAcceleration = ElevatorConstants.MAGIC_MOTION_ACCELERATION;    // rot/s²
        mm.MotionMagicJerk = ElevatorConstants.MAGIC_MOTION_JERK;           // rot/s³
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
