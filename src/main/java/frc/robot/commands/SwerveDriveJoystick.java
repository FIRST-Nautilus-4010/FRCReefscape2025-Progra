package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

public class SwerveDriveJoystick extends Command{
    private final Swerve swerve;
    private final Supplier<Double> x, y, z;
    private final Supplier<Boolean> field_relative;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;
    StructArrayPublisher<SwerveModuleState> swerve_desired_state_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("desiredStates", SwerveModuleState.struct).publish();

    public SwerveDriveJoystick(Swerve swerve, 
    Supplier<Double> x, Supplier<Double> y, Supplier<Double> z,
    Supplier<Boolean> field_relative){
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        this.field_relative = field_relative;
        this.xLimiter = new SlewRateLimiter(Constants.ChassisConstants.MAX_ACCEL);
        this.yLimiter = new SlewRateLimiter(Constants.ChassisConstants.MAX_ACCEL);
        this.zLimiter = new SlewRateLimiter(Constants.ChassisConstants.MAX_ANG_ACCEL);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double xSpeed = x.get();
        double ySpeed = y.get();
        double zSpeed = z.get();

        xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.JOYSTICK_DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.JOYSTICK_DEADZONE ? ySpeed : 0.0;
        zSpeed = Math.abs(zSpeed) > Constants.OperatorConstants.JOYSTICK_DEADZONE ? zSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.ChassisConstants.MAX_SPD;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.ChassisConstants.MAX_SPD;
        zSpeed = zLimiter.calculate(zSpeed) * Constants.ChassisConstants.MAX_ANG_SPD;

        ChassisSpeeds chassisSpeeds;
        if (field_relative.get()){
            //Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerve.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }

        SwerveModuleState[] moduleStates = Constants.ChassisConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        swerve.setStates(moduleStates);

        swerve_desired_state_publisher.set(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
