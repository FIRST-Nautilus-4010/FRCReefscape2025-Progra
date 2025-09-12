package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class SwerveModule {
    final SwerveController controller;
    final SwerveIO io;

    public SwerveModule(int driveTalonFxId, int turningSparkId, int absoluteEncoderId) {
        io = new SwerveIO(driveTalonFxId, turningSparkId, absoluteEncoderId);
        controller = new SwerveController(io.getDriveMotor());
    }

    // Returns the current state of the swerve module as a SwerveModuleState object.
    public SwerveModuleState getState() {
        double driveSpeed = io.getDriveMotorVelocity(); // <-- Converts the drive motor speed from RPM to meters per second.
        double turningPosition = io.getAbsoluteEncoderRad(); // <-- Retrieves the current turning position in radians.

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition)); // <-- Creates a SwerveModuleState with speed and rotation.
    }

    // Returns the current position of the swerve module as a SwerveModulePosition object.
    public SwerveModulePosition getPosition() {
        double driveDistance =io.getDriveMotorPosition(); // <-- Converts the drive motor encoder rotations to meters traveled.
        double turningPosition = io.getAbsoluteEncoderRad() * SwerveConstants.TURNING_ROT_2_RAD; // <-- Converts the turning encoder rotations to radians.

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition)); // <-- Creates a SwerveModulePosition with distance and rotation.
    }

    public void stop() {
        io.stop(); // <-- Stops both the drive and turning motors.
    }

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {
        // Avoid auto-aligning the swerve module when the robot is stationary.
        // If the desired speed is below a threshold (0.090 m/s), stop the module and exit the method.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.090) {
            io.stop(); // <-- Stops both the drive and turning motors.
            return; // <-- Exits the method to prevent unnecessary movement.
        }

        // Retrieves the current rotation of the module from the absolute encoder.
        Rotation2d encoderRotation = new Rotation2d(io.getAbsoluteEncoderRad()); // <-- Converts the encoder position to a Rotation2d object.

        // Optimizes the desired state to minimize unnecessary rotation.
        // Ensures the module rotates in the shortest direction to achieve the desired angle.
        desiredState.optimize(encoderRotation); // <-- Optimizes the desired state based on the current encoder rotation.

        // Adjusts the desired speed based on the cosine of the angle difference.
        // This compensates for the alignment of the module relative to the desired angle.
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos(); // <-- Scales the speed by the cosine of the angle difference.

        // Sets the drive motor speed as a fraction of the maximum speed.
        controller.setVelocity(desiredState.speedMetersPerSecond / SwerveConstants.ROT_2_M); // <-- Normalizes and sets the drive motor speed.

        // Calculates the output for the turning motor using a PID controller.
        // The PID controller adjusts the turning motor to achieve the desired angle.
        double output = controller.getPID(io.getAbsoluteEncoderRad(), desiredState.angle.getRadians()); // <-- PID calculation for turning motor.
        io.getTurningMotor().set(output); // <-- Sets the turning motor output based on the PID calculation.
    }
}