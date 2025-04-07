package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.utils.Constants.ChassisConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final SparkMax turningMotor;
    private final PIDController turningPIDController;
    private final int driveTalonFxId;
    private final int turningSparkId;
    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffset;


    public SwerveModule(int driveTalonFxId, int turningSparkId, int absoluteEncoder_id, double absoluteEncoderOffset, Boolean driveInverted, Boolean turningInverted) {
        this.driveTalonFxId = driveTalonFxId;
        this.turningSparkId = turningSparkId;

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        absoluteEncoder = new CANcoder(absoluteEncoder_id);

        driveMotor = new TalonFX(this.driveTalonFxId);
        turningMotor = new SparkMax(this.turningSparkId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig turningMotorConfig = new SparkMaxConfig();
        turningMotorConfig.inverted(turningInverted);

        TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
        if (driveInverted) {  
            driveMotorConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); 
        } 
        else {
            driveMotorConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); 
        };

        driveMotor.getConfigurator().apply(driveMotorConfigs);
        turningMotor.configure(turningMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
                
        // Assigns a pid controller for the turning motor. This one takes a P variable stablish in constants that specifies the proportional PID value
        turningPIDController = new PIDController(ModuleConstants.PID_P, ModuleConstants.PID_I, ModuleConstants.PID_D);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Set The encoders into 0 position
        resetEncoders();

    }

    // Stablish the encoders into 0 position
    public final void resetEncoders(){
        driveMotor.setPosition(0);
    }

    // Returns a SwerveModuleState object with the module state
    public SwerveModuleState getState() {
        double driveSpeed = driveMotor.get() * ModuleConstants.ENC_RPM_2_M_S;
        double turningPosition = getAbsoluteEncoderRad();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    // Returns a SwerveModulePosition object with the actual modules position
    public SwerveModulePosition getPosition() {
        double driveDistance = driveMotor.getPosition().getValue().magnitude() * ModuleConstants.ENC_ROT_2_M;
        double turningPosition = getAbsoluteEncoderRad() * ModuleConstants.TURNING_ROT_2_RAD;

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    // Stops the motors
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    // Returns the absolute encoder actual radians
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition().getValue().magnitude();
        angle *= 2 * Math.PI;
        angle += absoluteEncoderOffset;
        //SmartDashboard.putString("algo", angle.toString);
        return angle;
    }

    public ArrayList<Double> getInterPts(double angle) {
        // Calculate the turning motor positions
        double P0 = getAbsoluteEncoderRad();
        double P3 = angle;
        double n = Math.abs(P3 - P0);
        double k = 0.0186430923726995846 * n;
        P0 *= k;
        P3 *= k;
        
        
        for (int i = 0; i <= n; i++) {
            double t = i / n;
            double point = P0 + (P3 - P0) / (1 + Math.pow(Math.E, -8 * (t- 0.5)));

            points.add(point);
        }

        return points;
    }

    int i = 0;
    ArrayList<Double> points = new ArrayList<>();

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {
        // Avoid auto alining while the robot is being operate
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.09){
            stop();
            return;
        }

        Rotation2d encoderRotation = new Rotation2d(getAbsoluteEncoderRad() * ModuleConstants.TURNING_ROT_2_RAD);
        desiredState.optimize(encoderRotation);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();

        driveMotor.set(desiredState.speedMetersPerSecond / ChassisConstants.MAX_SPD);

        if (i == 0) {
            points = getInterPts(desiredState.angle.getRadians());
        }
            
        double desiredPos = points.get(i);
        if (Math.abs(desiredPos - getAbsoluteEncoderRad()) < .1) {
            i++;
        } else {
            turningMotor.set(turningPIDController.calculate(getAbsoluteEncoderRad(), desiredPos));
        }

        if (i >= points.size() - 1) {
            i = 0;
        }

        
    }
}