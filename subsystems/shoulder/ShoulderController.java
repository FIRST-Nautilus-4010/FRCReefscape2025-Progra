package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShoulderController {
    private final TalonFX leader;
    private final TalonFXConfiguration configuration;
    private final MotionMagicVoltage request;

    public ShoulderController(TalonFX leader) {
        this.leader = leader;

        request = new MotionMagicVoltage(0);

        configuration = new TalonFXConfiguration();

        setSlotGains();
        setMMSettings();

        leader.getConfigurator().apply(configuration);
    }

    private void setSlotGains() {
        var slot0Configs = configuration.Slot0;
        slot0Configs.kS = ShoulderConstants.SLOT0_KS;
        slot0Configs.kV = ShoulderConstants.SLOT0_KV;
        slot0Configs.kA = ShoulderConstants.SLOT0_KA;
        slot0Configs.kP = ShoulderConstants.SLOT0_KP;
        slot0Configs.kI = ShoulderConstants.SLOT0_KI;
        slot0Configs.kD = ShoulderConstants.SLOT0_KD;
    }

    private void setMMSettings() {
        var motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShoulderConstants.MAGIC_MOTION_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ShoulderConstants.MAGIC_MOTION_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShoulderConstants.MAGIC_MOTION_JERK;
    }

    public void goTo(double position) {
        leader.setControl(request.withPosition(position));
    }
}
