package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorController {
    private final TalonFX leader;
    private final TalonFXConfiguration configuration;
    private final MotionMagicVoltage request;

    public ElevatorController(TalonFX leader) {
        this.leader = leader;
        request = new MotionMagicVoltage(0);

        configuration = new TalonFXConfiguration();

        setSlotGains();
        setMMSettings();

        leader.getConfigurator().apply(configuration);
    }

    private void setSlotGains() {
        var slot0Configs = configuration.Slot0;
        slot0Configs.kS = ElevatorConstants.SLOT0_KS;
        slot0Configs.kV = ElevatorConstants.SLOT0_KV;
        slot0Configs.kA = ElevatorConstants.SLOT0_KA;
        slot0Configs.kP = ElevatorConstants.SLOT0_KP;
        slot0Configs.kI = ElevatorConstants.SLOT0_KI;
        slot0Configs.kD = ElevatorConstants.SLOT0_KD;
    }

    private void setMMSettings() {
        var motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MAGIC_MOTION_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MAGIC_MOTION_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MAGIC_MOTION_JERK;
    }

    public void goTo(double position) {
        leader.setControl(request.withPosition(position));
    }
}
