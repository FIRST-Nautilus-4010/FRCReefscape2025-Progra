package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.commands.SetState;

// Subsystem responsible for controlling the elevator mechanism
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io; // Handles inputs and outputs (motors, sensors)
    private final IntakeController controller;

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public IntakeSubsystem() {
        this.io = new IntakeIO();
        controller = new IntakeController(io.getMotorPosition(), io.getMotorVelocity());
    }

    public Command goToIntake() {
        return new SetState(10, 1, controller, () -> io.getRollersAmp(), this);
    }

    public Command save() {
        return new SetState(0, 16.01, controller, () -> 120.0, this);
    }

    public Command give() {
        return new SetState(-10, 16.01, controller, () -> 120.0, this);
    }
    
    public Command goToL1() {
        return new SetState(-10, 8, controller, () -> 120.0, this);
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake pos", io.getPos());
        SmartDashboard.putNumber("Intake vel", io.getVel());
        SmartDashboard.putNumber("Intake current", io.getRollersAmp());
        SmartDashboard.putString("IntakeState", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        
    }
}