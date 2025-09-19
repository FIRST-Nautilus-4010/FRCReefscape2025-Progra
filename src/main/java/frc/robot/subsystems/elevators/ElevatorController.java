package frc.robot.subsystems.elevators;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorController {
    private final TalonFX leader;
    private final TalonFXConfiguration configuration;
    private final MotionMagicExpoVoltage posRequest;

    public ElevatorController(TalonFX leader) {
        this.leader = leader;

        posRequest = new MotionMagicExpoVoltage(0).withSlot(0);

        configuration = new TalonFXConfiguration();

        setPositionSlotGains();
        setMMSettings();

        leader.getConfigurator().apply(configuration);
    }

    private void setPositionSlotGains() {
        var slot0 = configuration.Slot0;
        slot0.kG = ElevatorConstants.POS_KG;
        slot0.kS = ElevatorConstants.POS_KS;
        slot0.kV = ElevatorConstants.POS_KV;
        slot0.kA = ElevatorConstants.POS_KA;
        slot0.kP = ElevatorConstants.POS_KP;
        slot0.kI = ElevatorConstants.POS_KI;
        slot0.kD = ElevatorConstants.POS_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAGIC_MOTION_VELOCITY;
        motionMagic.MotionMagicAcceleration = ElevatorConstants.MAGIC_MOTION_ACCELERATION;
        motionMagic.MotionMagicJerk = ElevatorConstants.MAGIC_MOTION_JERK;
        motionMagic.MotionMagicExpo_kV = ElevatorConstants.MAGIC_MOTION_EXPO_KV; 
        motionMagic.MotionMagicExpo_kA = ElevatorConstants.MAGIC_MOTION_EXPO_KA;
    }

    public void moveTo(double height) {
        leader.setControl(posRequest.withPosition(height / ElevatorConstants.ROT_2_M));
    }
}