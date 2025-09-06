package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles inputs and outputs (motors, sensors)
    private final ElevatorController controller; // PID controllers for height and angle

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ElevatorSubsystem() {
        this.io = new ElevatorIO();
        this.controller = new ElevatorController(io.getLeaderMotor());
    }

    public Command goToL1() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L1_POS), this);
    }

    public Command goToL2() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L2_POS), this);
    }

    public Command goToL3() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L3_POS), this);
    }

    public Command goToL4() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.L4_POS), this);
    }

    public Command goToIntake() {
        return new InstantCommand(() -> controller.goTo(ElevatorConstants.INTAKE_POS), this);
    }

    public Command elevate() {
        return new InstantCommand(() -> controller.setVelocity(95), this);
    }

    public Command stop() {
        return new InstantCommand(() -> io.stop(), this);
    }

    public Command descend() {
        return new InstantCommand(() -> controller.setVelocity(-95), this);
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName(), io.getHeight());
    }
}


