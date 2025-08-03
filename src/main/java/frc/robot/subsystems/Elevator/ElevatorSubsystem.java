package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles inputs and outputs (motors, sensors)
    private final ElevatorController controller; // PID controllers for height and angle
    private final XboxController xboxController; // Operator joystick

    private ElevatorState currentState = ElevatorState.RUN_MANUAL;

    private double targetHeight = 0;
    private double targetAngle = 0;

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ElevatorSubsystem(XboxController xboxController) {
        this.xboxController = xboxController;
        this.io = new ElevatorIO();
        this.controller = new ElevatorController(io.getLeftLeaderMotor(), io.getRightLeaderMotor());
    }

    /**
     * Sets the current state of the elevator subsystem.
     * @param state The new state to set.
     */
    public void setState(ElevatorState state) {
        this.currentState = state;
    }

    /**
     * Sets the target height and angle for the elevator.
     * @param height The target height in meters.
     */
    public void setTargetHeight(double height) {
        targetHeight = height * ElevatorConstants.ROT_2_M; // Convert to meters
    }

    /**
     * Sets the target angle for the elevator arm.
     * @param angle The target angle in degrees.
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle * ElevatorConstants.ROT_2_RADIAN; // Convert to radians
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     * This method is called periodically to update the elevator's state
     * based on the current state and user inputs.
     * It handles different states such as running to a position, height, angle,
     */
    @Override
    public void periodic() {
        switch (currentState) {
            case RUN_TO_POSITION:
                controller.setTargetPosition(targetHeight, targetAngle);
                break;
            case RUN_TO_HEIGHT:
                
                break;
            case RUN_TO_ANGLE:
                
                break;
            case RUN_MANUAL:
                double heightPower = xboxController.getRightY(); 
                double anglePower = xboxController.getLeftX();

                controller.setPower(heightPower, anglePower);
                break;
        }

        controller.periodic();
    }
}


