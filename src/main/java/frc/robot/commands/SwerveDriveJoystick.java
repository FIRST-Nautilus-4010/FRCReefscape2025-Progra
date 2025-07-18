package frc.robot.commands;
// Imports from Java
import java.util.function.Supplier; 

// Imports from WPILib
import edu.wpi.first.math.filter.SlewRateLimiter; 
import edu.wpi.first.math.kinematics.ChassisSpeeds; 
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.networktables.NetworkTableInstance; 
import edu.wpi.first.networktables.StructArrayPublisher; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.Command; 

// Local imports
import frc.robot.subsystems.swerve.Swerve; 
import frc.robot.utils.Constants.ChassisConstants;

public class SwerveDriveJoystick extends Command {

    //--------Constants--------
    final double JOYSTICK_DEADZONE = .1; // <-- Deadzone for joystick inputs to ignore small movements.

    final Swerve swerve; // <-- Instance of the swerve subsystem.
    final Supplier<Double> x, y, z; // <-- Suppliers for joystick inputs (X, Y, Z axes).
    final Supplier<Boolean> fieldRelative; // <-- Supplier for field-relative mode.
    final SlewRateLimiter xLimiter, yLimiter, zLimiter; // <-- Rate limiters for joystick inputs.
    StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("desiredStates", SwerveModuleState.struct).publish(); // <-- Publishes desired swerve module states to NetworkTables.

    public SwerveDriveJoystick(
        Swerve swerve, Supplier<Double> x, 
        Supplier<Double> y, Supplier<Double> z,
        Supplier<Boolean> fieldRelative
    ) {
        this.swerve = swerve; // <-- Initializes the swerve subsystem.
        this.x = x; // <-- Initializes the X-axis joystick input supplier.
        this.y = y; // <-- Initializes the Y-axis joystick input supplier.
        this.z = z; // <-- Initializes the Z-axis joystick input supplier.
        this.fieldRelative = fieldRelative; // <-- Initializes the field-relative mode supplier.
        this.xLimiter = new SlewRateLimiter(ChassisConstants.MAX_ACCEL);
        this.yLimiter = new SlewRateLimiter(ChassisConstants.MAX_ACCEL);
        this.zLimiter = new SlewRateLimiter(ChassisConstants.MAX_ANG_ACCEL);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Updates the SmartDashboard with the robot's current position (X, Y coordinates).
        SmartDashboard.putNumberArray("Robot pos", new Double[] {swerve.getPose().getX(), swerve.getPose().getY()});

        // Retrieves joystick inputs for X, Y, and Z axes.
        double xSpeed = x.get(); // <-- Raw X-axis input from the joystick.
        double ySpeed = y.get(); // <-- Raw Y-axis input from the joystick.
        double zSpeed = z.get(); // <-- Raw Z-axis input (rotation) from the joystick.

        // Applies a deadzone to joystick inputs to ignore small movements.
        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0.0; // <-- Filters X-axis input.
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0.0; // <-- Filters Y-axis input.
        zSpeed = Math.abs(zSpeed) > JOYSTICK_DEADZONE ? zSpeed : 0.0; // <-- Filters Z-axis input.

        // Applies rate limiting to joystick inputs and scales them to maximum speeds.
        xSpeed = xLimiter.calculate(xSpeed) * ChassisConstants.MAX_SPD; // <-- Limits and scales X-axis speed.
        ySpeed = yLimiter.calculate(ySpeed) * ChassisConstants.MAX_SPD; // <-- Limits and scales Y-axis speed.
        zSpeed = zLimiter.calculate(zSpeed) * ChassisConstants.MAX_ANG_SPD; // <-- Limits and scales Z-axis angular speed.

        ChassisSpeeds chassisSpeeds;
        if (fieldRelative.get()) {
            // Converts joystick inputs to field-relative speeds using the robot's current rotation.
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerve.getRotation2d()); // <-- Field-relative speeds.
        } else {
            // Uses robot-relative speeds directly from joystick inputs.
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed); // <-- Robot-relative speeds.
        }

        // Converts chassis speeds to individual swerve module states.
        SwerveModuleState[] moduleStates = ChassisConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds); // <-- Calculates swerve module states.

        // Sets the desired states for the swerve modules.
        swerve.setStates(moduleStates); // <-- Updates swerve modules with calculated states.

        // Publishes the desired swerve module states to NetworkTables for debugging or visualization.
        swerveDesiredStatePublisher.set(moduleStates); // <-- Publishes module states.
    }

    @Override
    public void end(boolean interrupted) {
        // Stops all swerve modules when the command ends or is interrupted.
        swerve.stopModules(); // <-- Stops swerve modules.
    }
}
