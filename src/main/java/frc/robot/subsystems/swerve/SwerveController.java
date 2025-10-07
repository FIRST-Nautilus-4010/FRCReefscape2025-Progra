package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SwerveController {
    private final TalonFX driveMotor;
    private final TalonFXConfiguration configuration;
    private final MotionMagicVelocityVoltage velRequest;

    private final SparkMax turningMotor;
    private final SparkClosedLoopController turningPIDController;
    private final SparkMaxConfig config;

    public SwerveController(TalonFX driveMotor, SparkMax turningMotor) {
        this.driveMotor = driveMotor;
        configuration = new TalonFXConfiguration();
        velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

        this.turningMotor = turningMotor;
        config = new SparkMaxConfig();
        this.turningPIDController = turningMotor.getClosedLoopController();


        setVelocitySlotGains();
        setMMSettings();

        setStearingSlot();
        setMaxMotionParameters();;

        this.driveMotor.getConfigurator().apply(configuration);
        this.turningMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setStearingSlot() {
        config.closedLoop
        .p(SwerveConstants.MAX_MOTION_KP)
        .i(SwerveConstants.MAX_MOTION_KI)
        .d(SwerveConstants.MAX_MOTION_KD)
        .outputRange(SwerveConstants.MAX_MOTION_MIN_OUTPUT, SwerveConstants.MAX_MOTION_MAX_OUTPUT)
        .positionWrappingInputRange(-Math.PI / SwerveConstants.STR_RATIO, Math.PI / SwerveConstants.STR_RATIO)
        .positionWrappingEnabled(true);
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
    }

    private void setMaxMotionParameters() {
        config.closedLoop.maxMotion
        .maxVelocity(SwerveConstants.MAX_MOTION_VEL) // cambiar a cruiseVelocity
        .maxAcceleration(SwerveConstants.MAX_MOTION_ACC)
        .allowedClosedLoopError(SwerveConstants.MAX_MOTION_ALLOWED_ERR)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    }

    public void setVelocity(double velocity) {
        double adjustedVelocity = velocity / SwerveConstants.ROT_2_M;
        driveMotor.setControl(velRequest.withVelocity(adjustedVelocity));
    }

    public void setAngle(double angle) {
        turningPIDController.setReference((angle / SwerveConstants.ROT_2_RAD), ControlType.kMAXMotionPositionControl);
    }
}
