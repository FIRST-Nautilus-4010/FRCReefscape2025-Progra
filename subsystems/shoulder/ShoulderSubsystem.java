// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
 private final ShoulderIO io; // Handles inputs and outputs (motors, sensors)
    private final ShoulderController controller; // PID controllers for height and angle

    /**
     * Constructs the ElevatorSubsystem with the given Xbox controller.
     * @param xboxController The Xbox controller used for manual control.
     */
    public ShoulderSubsystem() {
        this.io = new ShoulderIO();
        this.controller = new ShoulderController(io.getLeaderMotor());
    }

    public Command goToL1() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L1_POS), this);
    }

    public Command goToL2() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L2_POS), this);
    }

    public Command goToL3() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L3_POS), this);
    }

    public Command goToL4() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.L4_POS), this);
    }

    public Command goToIntake() {
        return new InstantCommand(() -> controller.goTo(ShoulderConstants.INTAKE_POS), this);
    }

    public Command elevate() {
        return new InstantCommand(() -> io.setVoltage(11.2), this);
    }

    public Command stop() {
        return new InstantCommand(() -> io.stop(), this);
    }

    public Command descend() {
        return new InstantCommand(() -> io.setVoltage(-11.2), this);
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName(), io.getAngle());
    }
}
