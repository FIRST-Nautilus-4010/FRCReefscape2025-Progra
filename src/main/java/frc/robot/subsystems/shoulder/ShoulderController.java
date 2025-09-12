package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShoulderController {
    private final TalonFX leader;
    private final TalonFXConfiguration configuration;
    private final MotionMagicExpoVoltage posRequest;
    private final MotionMagicVelocityVoltage velRequest;

    public ShoulderController(TalonFX leader) {
        this.leader = leader;

        posRequest = new MotionMagicExpoVoltage(0).withSlot(0);
        velRequest = new MotionMagicVelocityVoltage(0).withSlot(1);

        configuration = new TalonFXConfiguration();

        setPositionSlotGains();
        setVelocitySlotGains();
        setMMSettings();

        leader.getConfigurator().apply(configuration);
    }

    private void setPositionSlotGains() {
        var slot0 = configuration.Slot0;
        slot0.kS = ShoulderConstants.POS_KS;
        slot0.kV = ShoulderConstants.POS_KV;
        slot0.kA = ShoulderConstants.POS_KA;
        slot0.kP = ShoulderConstants.POS_KP;
        slot0.kI = ShoulderConstants.POS_KI;
        slot0.kD = ShoulderConstants.POS_KD;
    }

    private void setVelocitySlotGains() {
        var slot1 = configuration.Slot1;
        slot1.kS = ShoulderConstants.VEL_KS;
        slot1.kV = ShoulderConstants.VEL_KV;
        slot1.kA = ShoulderConstants.VEL_KA;
        slot1.kP = ShoulderConstants.VEL_KP;
        slot1.kI = ShoulderConstants.VEL_KI;
        slot1.kD = ShoulderConstants.VEL_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicCruiseVelocity = ShoulderConstants.MAGIC_MOTION_VELOCITY;
        motionMagic.MotionMagicAcceleration = ShoulderConstants.MAGIC_MOTION_ACCELERATION;
        motionMagic.MotionMagicJerk = ShoulderConstants.MAGIC_MOTION_JERK;
        motionMagic.MotionMagicExpo_kV = ShoulderConstants.MAGIC_MOTION_EXPO_KV; 
        motionMagic.MotionMagicExpo_kA = ShoulderConstants.MAGIC_MOTION_EXPO_KA;
    }

    public void goTo(double position) {
        leader.setControl(posRequest.withPosition(position));
    }

    public void setVelocity(double velocity) {
        leader.setControl(velRequest.withVelocity(velocity));
    }
}