package frc.robot.subsystems.elevators;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevators.commands.MoveTo;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles inputs and outputs (motors, sensors)

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ElevatorSubsystem() {
        this.io = new ElevatorIO();
    }

    private Command moveTo(double height, Supplier<Double> shoulderAngle) {
        return new MoveTo(height, shoulderAngle, io, this);
    }

    public Command moveToL1(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L1_POS, shoulderAngle).withName("Elevator to L1");
    }

    public Command moveToL2(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L2_POS, shoulderAngle).withName("Elevator to L2");
    }

    public Command moveToL3(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L3_POS, shoulderAngle).withName("Elevator to L3");
    }

    public Command moveToL4(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L4_POS, shoulderAngle).withName("Elevator to L4");
    }

    public Command moveToIntake(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.INTAKE_POS, shoulderAngle).withName("Elevator to Intake");
    }

    public Command moveToTravel(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.TRAVEL_POS, shoulderAngle).withName("Elevator to Travel");
    }

    public Command placeL2(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L2_POS - ElevatorConstants.PUT_L2L3_POS, shoulderAngle).withName("Elevator place L2");
    }

    public Command placeL3(Supplier<Double> shoulderAngle) {
        return moveTo(ElevatorConstants.L3_POS - ElevatorConstants.PUT_L2L3_POS, shoulderAngle).withName("Elevator place L3");
    }

    public double getHeight() {
        return io.getHeight();
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", io.getHeight());
        SmartDashboard.putString("ElevatorState", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        
        if (
            (io.getHeight() >= ElevatorConstants.MAX_HEIGHT && io.getVelocity() > 0) || 
            (io.getHeight() <= 0 && io.getVelocity() < 0)
        ) {
            io.stop();
        }
        
    }
}