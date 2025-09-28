package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveIO {
    private final TalonFX driveMotor;
    private final SparkMax turningMotor;
    private final CANcoder absoluteEncoder;

    public SwerveIO(int driveTalonFxId, int turningSparkId, int absoluteEncoderId) {

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveTalonFxId);
        turningMotor = new SparkMax(turningSparkId, MotorType.kBrushless);

        // Set The encoders into 0 position
        resetEncoders();

    }

    // Resets the drive motor encoder position to zero.
    public final void resetEncoders() {
        turningMotor.getEncoder().setPosition((getAbsoluteEncoderRad() / (2 * Math.PI)) * SwerveConstants.STR_RATIO);
        driveMotor.setPosition(0); // <-- Sets the drive motor encoder position to zero.
    }


    // Stops both the drive and turning motors of the swerve module.
    public void stop() {
        driveMotor.stopMotor(); // <-- Stops the drive motor by setting its speed to zero.
        turningMotor.stopMotor();; // <-- Stops the turning motor by setting its speed to zero.
    }

    // Returns the absolute encoder position in radians, adjusted by the offset.
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue().magnitude(); // <-- Retrieves the raw absolute encoder position.
        angle *= 2 * Math.PI; // <-- Converts the encoder position to radians.

        return angle; // <-- Returns the adjusted encoder position in radians.
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getTurningMotor() {
        return turningMotor;
    }

    public double getDriveMotorVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.ROT_2_M; // <-- Converts the drive motor speed from RPM to meters per second.
    }

    public double getDriveMotorPosition() {
        return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.ROT_2_M; // <-- Converts the drive motor encoder rotations to meters traveled.
    }
}
