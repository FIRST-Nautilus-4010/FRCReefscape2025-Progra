// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.shoulder.Commands.PutCommand;

public class ShoulderSubsystem extends SubsystemBase {
    private final ShoulderIO io; // Handles inputs and outputs (motors, sensors)
    private final ShoulderController controller; // PID controllers for height and angle
    private final ElevatorIO elevatorIO;

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ShoulderSubsystem(ElevatorIO elevatorIO) {
        this.io = new ShoulderIO();
        this.controller = new ShoulderController(io.getLeaderMotor());
        this.elevatorIO = elevatorIO;
    }

    public Command goToL1() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L1_POS), this).withName("ShoulderL1Cmd");
    }

    public Command goToL2() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L2_POS), this).withName("ShoulderL2Cmd");
    }

    public Command goToL3() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L3_POS), this).withName("ShoulderL3Cmd");
    }

    public Command goToL4() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L4_POS), this).withName("ShoulderL4Cmd");
    }

    public Command goToIntake() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.INTAKE_POS), this).withName("ShoulderIntakeCmd");
    }

    public Command spinRight() {
        return new InstantCommand(() -> controller.setVelocity(1), this).withName("ShoulderSpinRightCmd");
    }

    public Command stop() {
        return new InstantCommand(() -> io.stop(), this).withName("ShoulderStopCmd");
    }

    public Command spinLeft() {
        return new InstantCommand(() -> controller.setVelocity(-1), this).withName("ShoulderSpinLeftCmd");
    }

    public Command put() {
        return new PutCommand(this, controller, io);
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName(), io.getAngle());

        if (
            (io.getAngle() >= ShoulderConstants.MAX_ANGLE && io.getVelocity() < 0) || 
            (io.getAngle() <= ShoulderConstants.MIN_ANGLE && io.getVelocity() > 0) ||
            (elevatorIO.getHeight() <= ShoulderConstants.SAFETY_HEIGHT)
        ) {
            io.stop();
        }
    }
}
