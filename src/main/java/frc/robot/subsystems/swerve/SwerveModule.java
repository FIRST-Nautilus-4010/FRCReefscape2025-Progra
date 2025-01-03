package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder turningEncoder;
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
        turningMotor = new CANSparkMax(this.turningSparkId, MotorType.kBrushless);

        turningEncoder = turningMotor.getEncoder();

        driveMotor.setInverted(driveInverted);
        turningMotor.setInverted(turningInverted);
        
        // Converts the encoder rotations into common units like meters per second
        turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.TURNING_ROT_2_RAD);
        turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.TURNING_RPM_2_RAD_S);

        // Assigns a pid controller for the turning motor. This one takes a P variable stablish in constants that specifies the proportional PID value
        turningPIDController = new PIDController(Constants.ModuleConstants.PID_P, Constants.ModuleConstants.PID_I, Constants.ModuleConstants.PID_D);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Set The encoders into 0 position
        resetEncoders();

    }

    // Stablish the encoders into 0 position
    public final void resetEncoders(){
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad()); // Calibrate the turning encoder with the absolute encoder
    }

    // Returns a SwerveModuleState object with the module state
    public SwerveModuleState getState() {
        double driveSpeed = driveMotor.get() * Constants.ModuleConstants.ENC_RPM_2_M_S;
        double turningPosition = turningEncoder.getPosition();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    // Returns a SwerveModulePosition object with the actual modules position
    public SwerveModulePosition getPosition() {
        double driveDistance = driveMotor.getPosition().getValue() * Constants.ModuleConstants.ENC_ROT_2_M;
        double turningPosition = turningEncoder.getPosition();

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    // Stops the motors
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    // Returns the absolute encoder actual radians
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        angle *= 2 * Math.PI;
        angle += absoluteEncoderOffset;
        //SmartDashboard.putString("algo", angle.toString);
        return angle;
    }

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {

        // Avoid auto alining while the robot is being operate
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.09){
            stop();
            return;
        }

        // The actual turning encoder rotation
        var encoder_rotation = new Rotation2d(turningEncoder.getPosition());

        // Optimize the angle for avoiding more than 90 degrees movements
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoder_rotation);

        state.speedMetersPerSecond *= state.angle.minus(encoder_rotation).getCos();
        
        // The actual turningMotor_position
        double turningMotor_position = turningEncoder.getPosition();

        // Assigns a speed to the drive motor
        driveMotor.set(state.speedMetersPerSecond / Constants.ChassisConstants.MAX_SPD);

        // Calculates the necessary set speed for turning the turning motor the specified angle
        turningMotor.set(turningPIDController.calculate(turningMotor_position, state.angle.getRadians()));
        //SmartDashboard.putNumber("PID", absoluteEncoder.getDeviceID());

        // Prints the swerve status
        SmartDashboard.putString("Debug", "absolute encoder angle" + absoluteEncoder.getAbsolutePosition().getValue().toString() + "id: " + absoluteEncoder.getDeviceID());


        
    }
}