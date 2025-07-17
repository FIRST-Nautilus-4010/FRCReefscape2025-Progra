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

    // Resets the drive motor encoder position to zero.
    public final void resetEncoders() {
        driveMotor.setPosition(0); // <-- Sets the drive motor encoder position to zero.
    }

    // Returns the current state of the swerve module as a SwerveModuleState object.
    public SwerveModuleState getState() {
        double driveSpeed = driveMotor.get() * ModuleConstants.ENC_RPM_2_M_S; // <-- Converts the drive motor speed from RPM to meters per second.
        double turningPosition = getAbsoluteEncoderRad(); // <-- Retrieves the current turning position in radians.

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition)); // <-- Creates a SwerveModuleState with speed and rotation.
    }

    // Returns the current position of the swerve module as a SwerveModulePosition object.
    public SwerveModulePosition getPosition() {
        double driveDistance = driveMotor.getPosition().getValue().magnitude() * ModuleConstants.ENC_ROT_2_M; // <-- Converts the drive motor encoder rotations to meters traveled.
        double turningPosition = getAbsoluteEncoderRad() * ModuleConstants.TURNING_ROT_2_RAD; // <-- Converts the turning encoder rotations to radians.

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition)); // <-- Creates a SwerveModulePosition with distance and rotation.
    }

    // Stops both the drive and turning motors of the swerve module.
    public void stop() {
        driveMotor.set(0.0); // <-- Stops the drive motor by setting its speed to zero.
        turningMotor.set(0); // <-- Stops the turning motor by setting its speed to zero.
    }

    // Returns the absolute encoder position in radians, adjusted by the offset.
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue().magnitude(); // <-- Retrieves the raw absolute encoder position.
        angle *= 2 * Math.PI; // <-- Converts the encoder position to radians.
        angle += absoluteEncoderOffset; // <-- Adds the offset to the encoder position for calibration.
        //SmartDashboard.putString("Algo", angle.toString); // <-- Debugging line (commented out).
        return angle; // <-- Returns the adjusted encoder position in radians.
    }

    public ArrayList<Double> getInterPts(double angle) {
        // Calculate the turning motor positions
        double P0 = getAbsoluteEncoderRad();
        double P3 = angle;
        double n = Math.round(Math.abs(P3 - P0)) * 10;
        
        System.out.println("Iniciando nuevo array de puntos con n :" + n + ", P3: " + P3 + " y P0: " + P0);
        for (int i = 0; i <= n; i++) {
            double t = i;
            double point = P0 + (t / n) * (P3 - P0);

            points.add(point);

            System.out.println(point);
        }
        
        return points;
    }

    int i = -1;
    ArrayList<Double> points = new ArrayList<>();

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {
        // Avoid auto-aligning the swerve module when the robot is stationary.
        // If the desired speed is below a threshold (0.090 m/s), stop the module and exit the method.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.090) {
            stop(); // <-- Stops both the drive and turning motors.
            return; // <-- Exits the method to prevent unnecessary movement.
        }

        // Retrieves the current rotation of the module from the absolute encoder.
        Rotation2d encoderRotation = new Rotation2d(getAbsoluteEncoderRad()); // <-- Converts the encoder position to a Rotation2d object.

        // Optimizes the desired state to minimize unnecessary rotation.
        // Ensures the module rotates in the shortest direction to achieve the desired angle.
        desiredState.optimize(encoderRotation); // <-- Optimizes the desired state based on the current encoder rotation.

        // Adjusts the desired speed based on the cosine of the angle difference.
        // This compensates for the alignment of the module relative to the desired angle.
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos(); // <-- Scales the speed by the cosine of the angle difference.

        // Sets the drive motor speed as a fraction of the maximum speed.
        driveMotor.set(desiredState.speedMetersPerSecond / ChassisConstants.MAX_SPD); // <-- Normalizes and sets the drive motor speed.

        // Calculates the output for the turning motor using a PID controller.
        // The PID controller adjusts the turning motor to achieve the desired angle.
        double output = turningPIDController.calculate(getAbsoluteEncoderRad(), desiredState.angle.getRadians()); // <-- PID calculation for turning motor.
        turningMotor.set(output); // <-- Sets the turning motor output based on the PID calculation.
    }
}