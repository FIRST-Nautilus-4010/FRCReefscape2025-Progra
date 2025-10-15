package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.elevators.ElevatorConstants;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SwerveController {
    private final TalonFX driveMotor;
    private final TalonFXConfiguration configuration;
    private final MotionMagicVelocityVoltage velRequest;

    private final TalonFX turningMotor;
    private final TalonFXConfiguration turningConfig;
    private final MotionMagicExpoVoltage posRequest;

    public SwerveController(TalonFX driveMotor, TalonFX turningMotor) {
        this.driveMotor = driveMotor;
        configuration = new TalonFXConfiguration();
        velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

        this.turningMotor = turningMotor;
        turningConfig = new TalonFXConfiguration();
        posRequest = new MotionMagicExpoVoltage(0).withSlot(0);


        setVelocitySlotGains();
        setMMSettings();

        setStearingSlot();

        this.driveMotor.getConfigurator().apply(configuration);
        this.turningMotor.getConfigurator().apply(turningConfig);
    }

    private void setStearingSlot() {
        var slot0 = turningConfig.Slot0;
        slot0.kG = SwerveConstants.POS_KG;
        slot0.kS = SwerveConstants.POS_KS;
        slot0.kV = SwerveConstants.POS_KV;
        slot0.kA = SwerveConstants.POS_KA;
        slot0.kP = SwerveConstants.POS_KP;
        slot0.kI = SwerveConstants.POS_KI;
        slot0.kD = SwerveConstants.POS_KD;
    }

    private void setVelocitySlotGains() {
        var slot0 = configuration.Slot0;
        slot0.kS = SwerveConstants.VEL_KS;
        slot0.kV = SwerveConstants.VEL_KV;
        slot0.kA = SwerveConstants.VEL_KA;
        slot0.kP = SwerveConstants.VEL_KP;
        slot0.kI = SwerveConstants.VEL_KI;
        slot0.kD = SwerveConstants.VEL_KD;
    }

    private void setMMSettings() {
        var motionMagic = configuration.MotionMagic;
        motionMagic.MotionMagicAcceleration = SwerveConstants.MAGIC_MOTION_ACC;
        motionMagic.MotionMagicJerk = SwerveConstants.MAGIC_MOTION_JERK;

        var motionMagicStr = turningConfig.MotionMagic;
        motionMagicStr.MotionMagicCruiseVelocity = SwerveConstants.MAGIC_MOTION_VELOCITY_STR;
        motionMagicStr.MotionMagicAcceleration = SwerveConstants.MAGIC_MOTION_ACCELERATION_STR;
        motionMagicStr.MotionMagicJerk = SwerveConstants.MAGIC_MOTION_JERK_STR;
        motionMagicStr.MotionMagicExpo_kV = SwerveConstants.MAGIC_MOTION_EXPO_KV_STR; 
        motionMagicStr.MotionMagicExpo_kA = SwerveConstants.MAGIC_MOTION_EXPO_KA_STR;
    }

    public void setVelocity(double velocity) {
        double adjustedVelocity = velocity / SwerveConstants.ROT_2_M;
        driveMotor.setControl(velRequest.withVelocity(adjustedVelocity));
    }

    public void setAngle(double angle) {
        double rotations = (angle / SwerveConstants.ROT_2_RAD);
        turningMotor.setControl(posRequest.withPosition(rotations));
    }
}
