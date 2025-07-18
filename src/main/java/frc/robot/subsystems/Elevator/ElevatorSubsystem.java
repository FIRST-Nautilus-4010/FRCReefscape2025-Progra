package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorSensors sensors;
    private final ElevatorController controller;
    private final Joystick joystick;

    private ElevatorState currentState = ElevatorState.RUN_MANUAL;

    public ElevatorSubsystem(Joystick joystick) {
        this.joystick = joystick;
        this.io = new ElevatorIO();
        this.sensors = new ElevatorSensors(io.getLeftMotor(), io.getRightMotor());
        this.controller = new ElevatorController();
    }

    public void setState(ElevatorState state) {
        this.currentState = state;
    }

    public void setTargetHeight(double height) {
        controller.setTargetHeight(height);
    }

    public void setTargetAngle(double angle) {
        controller.setTargetAngle(angle);
    }

    @Override
    public void periodic() {
        double heightPower = 0, anglePower = 0;

        switch (currentState) {
            case RUN_TO_POSITION:
                heightPower = controller.calculateHeight(sensors.getHeight());
                anglePower = controller.calculateAngle(sensors.getAngle());
                break;
            case RUN_TO_HEIGHT:
                heightPower = controller.calculateHeight(sensors.getHeight());
                anglePower = joystick.getX();
                break;
            case RUN_TO_ANGLE:
                anglePower = controller.calculateAngle(sensors.getAngle());
                heightPower = joystick.getY();
                break;
            case RUN_MANUAL:
            default:
                heightPower = joystick.getY();
                anglePower = joystick.getX();
        }

        io.setMotorPowers(heightPower + anglePower, heightPower - anglePower);
    }
}

