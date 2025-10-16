package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.swerve.SwerveConstants;

public class IntakeController {
    private final TalonFX motorPosition;
    private final TalonFX motorVelocity;
    private final TalonFXConfiguration configurationPosition;
    private final TalonFXConfiguration configurationVelocity;
    private final MotionMagicExpoVoltage magicPosition; 
    private final MotionMagicVelocityVoltage magicVelocity;

    public IntakeController(TalonFX motorPosition, TalonFX motorVelocity){
        this.motorPosition = motorPosition;
        this.motorVelocity = motorVelocity;

        configurationPosition = new TalonFXConfiguration();
        configurationVelocity = new TalonFXConfiguration();

        magicPosition = new MotionMagicExpoVoltage(0).withSlot(0);
        magicVelocity = new MotionMagicVelocityVoltage(0).withSlot(0);

        setPoseSlotGains();
        setVelSlotGains();

        setMMSettings();

        motorPosition.getConfigurator().apply(configurationPosition);
        motorVelocity.getConfigurator().apply(configurationVelocity);

    } 

    private void setPoseSlotGains(){
        var slot0 = configurationPosition.Slot0;
        slot0.kG = IntakeConstants.POS_KG;
        slot0.kS = IntakeConstants.POS_KS;
        slot0.kV = IntakeConstants.POS_KV;
        slot0.kA = IntakeConstants.POS_KA;
        slot0.kP = IntakeConstants.POS_KP;
        slot0.kI = IntakeConstants.POS_KI;
        slot0.kD = IntakeConstants.POS_KD;
    }

    private void setVelSlotGains(){
        var slot0 = configurationVelocity.Slot0;
        slot0.kS = SwerveConstants.VEL_KS;
        slot0.kV = SwerveConstants.VEL_KV;
        slot0.kA = SwerveConstants.VEL_KA;
        slot0.kP = SwerveConstants.VEL_KP;
        slot0.kI = SwerveConstants.VEL_KI;
        slot0.kD = SwerveConstants.VEL_KD;
    }

    private void setMMSettings() {
        var motionMagic = configurationPosition.MotionMagic;
        motionMagic.MotionMagicAcceleration = IntakeConstants.MAGIC_MOTION_ACC;
        motionMagic.MotionMagicJerk = IntakeConstants.MAGIC_MOTION_JERK;

        var motionMagicStr = configurationVelocity.MotionMagic;
        motionMagicStr.MotionMagicCruiseVelocity = IntakeConstants.MAGIC_MOTION_VELOCITY_STR;
        motionMagicStr.MotionMagicAcceleration = IntakeConstants.MAGIC_MOTION_ACCELERATION_STR;
        motionMagicStr.MotionMagicJerk = IntakeConstants.MAGIC_MOTION_JERK_STR;
        motionMagicStr.MotionMagicExpo_kV = IntakeConstants.MAGIC_MOTION_EXPO_KV_STR; 
        motionMagicStr.MotionMagicExpo_kA = IntakeConstants.MAGIC_MOTION_EXPO_KA_STR;
    }

    public void goTo(double pose){
        motorPosition.setControl(magicPosition.withPosition(pose));
    }

    public void setVelocity(double vel){
        motorVelocity.setControl(magicVelocity.withVelocity(vel));
    }
}
