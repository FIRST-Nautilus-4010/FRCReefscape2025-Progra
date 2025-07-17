package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Constants.ElevatorConstants.ElevatorState;

public class Elevator extends SubsystemBase {
    Joystick joystick; // <-- Joystick for manual control

    TalonFX krakenL; // <-- Left elevator motor
    TalonFX krakenR; // <-- Right elevator motor
    DutyCycleEncoder angleEnc; // <-- Angle encoder

    ProfiledPIDController anglePID; // <-- PID controller for angle control
    ProfiledPIDController heightPID; // <-- PID controller for height control
    
    // Elevator state enumeration
    private ElevatorState currentState = ElevatorState.RUN_MANUAL;

    public Elevator() {
        // Configure the left elevator motor
        anglePID = new ProfiledPIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D, ArmConstants.PROFILE_CONSTRAITS);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        angleEnc  = new DutyCycleEncoder(2);

        krakenL = new TalonFX(16);

        // Configure the right elevator motor
        heightPID = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, ElevatorConstants.PROFILE_CONSTRAITS);

        krakenR = new TalonFX(17);
    }

    public Elevator(Joystick joystick) {
        // Configure the left elevator motor
        anglePID = new ProfiledPIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D, ArmConstants.PROFILE_CONSTRAITS);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        angleEnc  = new DutyCycleEncoder(2);

        krakenL = new TalonFX(16);

        // Configure the right elevator motor
        heightPID = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, ElevatorConstants.PROFILE_CONSTRAITS);

        krakenR = new TalonFX(17);

        // Initialize the joystick for manual control
        this.joystick = joystick; // <-- Cast Joystick to int for the constructor
    }

    // Set the target height for the elevator
    public void setTargetHeight(double height) {
        heightPID.setGoal(height);
    }

    // Set the target angle for the elevator
    public void setTargetAngle(double angle) {
        anglePID.setGoal(angle);
    }

    // Set the current state of the elevator
    public void setCurrentState(ElevatorState state) {
        this.currentState = state;
    }

    // Get the current height of the elevator
    public double getCurrentHeight() {
        // Get the current positions of the right and left elevator motors
        double rightPos = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M; // <-- Convert to meters
        double leftPos = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M;

        return Math.abs(rightPos - leftPos); // <-- Return the absolute difference between the two positions
    }

    // Get the current angle of the elevator
    public double getCurrentAngle() {
        double rightPos = krakenR.getPosition().getValue().magnitude() * ArmConstants.ROT_2_RADIAN; // <-- Convert to radians
        double leftPos = krakenL.getPosition().getValue().magnitude() * ArmConstants.ROT_2_RADIAN;

        return Math.abs(rightPos + leftPos); // <-- Return the absolute sum of the two positions
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        double anglePwr; // <-- Power for the left elevator motor
        double heightPwr; // <-- Power for the right elevator motor

        // Check the current state of the elevator and perform actions accordingly
        switch (currentState) {
            case RUN_TO_POSITION:

                // Calculate the power for the motors based on the height PID
                anglePwr = anglePID.calculate(getCurrentAngle());
                heightPwr = heightPID.calculate(getCurrentHeight());

                break;
            
            case RUN_TO_ANGLE:
                // Calculate the power for the motors based on the angle PID
                anglePwr = anglePID.calculate(getCurrentAngle());
                heightPwr = joystick.getY();
                break;
            case RUN_TO_HEIGHT:
                // Calculate the power for the motors based on the height PID
                heightPwr = heightPID.calculate(getCurrentHeight());
                anglePwr = joystick.getX();
                break;
            case RUN_MANUAL:
                // Manual control using the joystick
                heightPwr = joystick.getY(); // <-- Use the Y axis for height control
                anglePwr = joystick.getX(); // <-- Use the X axis for angle control
                break;
            default:
                heightPwr = 0; // <-- Default power is 0 if no state is set
                anglePwr = 0;
                break;
        }

        // Sum both powers to get the final power for each side
        double leftPwr = heightPwr + anglePwr;
        double rightPwr = heightPwr - anglePwr;

        // Set the power to the motors
        krakenL.set(leftPwr);
        krakenR.set(rightPwr);
    }
}
