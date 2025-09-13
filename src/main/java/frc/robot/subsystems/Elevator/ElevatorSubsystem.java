package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.shoulder.ShoulderIO;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles inputs and outputs (motors, sensors)
    private final ShoulderIO shoulderIO;
    private final ElevatorController controller; // PID controllers for height and angle

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ElevatorSubsystem() {
        this.io = new ElevatorIO();
        this.shoulderIO = new ShoulderIO();
        this.controller = new ElevatorController(io.getLeaderMotor());
    }

    public Command goToL1() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L1_POS), this).withName("ElevatorL1Cmd");
    }

    public Command goToL2() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L2_POS), this).withName("ElevatorL2Cmd");
    }

    public Command goToL3() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L3_POS), this).withName("ElevatorL3Cmd");
    }

    public Command goToL4() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L4_POS), this).withName("ElevatorL4Cmd");
    }

    public Command goToIntake() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.INTAKE_POS), this).withName("ElevatorIntakeCmd");
    }

    public Command elevate() {

        return new InstantCommand(() -> controller.setVelocity(3), this).withName("ElevatorElevateCmd");
    }

    public Command descend() {
        return new InstantCommand(() -> controller.setVelocity(-3), this).withName("ElevatorDescendCmd");
    }

    public Command stop() {
        return new InstantCommand(() -> io.stop(), this).withName("ElevatorStopCmd");
    }

    public ElevatorIO getIO() {
        return io;
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName(), io.getHeight());

        if (
            (io.getHeight() >= ElevatorConstants.MAX_HEIGHT && io.getVelocity() > 0) || 
            (io.getHeight() <= 0 && io.getVelocity() < 0) ||
            (io.getHeight() <= ShoulderConstants.SAFETY_HEIGHT && Math.abs(shoulderIO.getAngle()) <= 0.1 && io.getVelocity() < 0)
        ) {
            io.stop();
        }
    }
}


