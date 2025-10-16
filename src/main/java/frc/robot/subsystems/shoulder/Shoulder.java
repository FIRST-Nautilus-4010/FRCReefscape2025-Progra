// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shoulder.commands.RotateTo;

public class Shoulder extends SubsystemBase {
    private final ShoulderIO io; // Handles inputs and outputs (motors, sensors)

    public Shoulder() {
        this.io = new ShoulderIO();
    }

    private Command rotateTo(double angle, Supplier<Double> pivotHeight) {
        return new RotateTo(angle, pivotHeight, io, this);
    }

    public Command rotateToL1(Supplier<Double> pivotHeight) {
        return rotateTo(ShoulderConstants.L1_POS, pivotHeight).withName("Shoulder to L1");
    }

    public Command rotateToL3(Supplier<Double> pivotHeight) {
        return rotateTo(-29.55, pivotHeight).withName("Shoulder to L2/L3");
    }

    public Command rotateToL4(Supplier<Double> pivotHeight) {
        return rotateTo(-32.76, pivotHeight).withName("Shoulder to L4");
    }

    public Command rotateToIntake(Supplier<Double> pivotHeight) {
        return rotateTo(ShoulderConstants.INTAKE_POS, pivotHeight);
    }

    public Command rotateToTravel(Supplier<Double> pivotHeight) {
        return rotateTo(ShoulderConstants.TRAVEL_POS, pivotHeight);
    }

    public Command placeL4(Supplier<Double> pivotHeight) {
        return rotateTo(-30.84, pivotHeight);
    }

    public Command placeL3(Supplier<Double> pivotHeight) {
        return rotateTo(-26.1079, pivotHeight);
    }

    public ShoulderIO getIO() {
        return io;
    }

    public double getAngle() {
        return io.getAngle();
    }

    public double getHeight() {
        return io.getHeight();
    }

    /**
     * Runs the periodic tasks for the elevator subsystem.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShoulderAngle", io.getAngle());
        SmartDashboard.putString("ShoulderState", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

        if (
            (io.getAngle() >= ShoulderConstants.MAX_ANGLE && io.getVelocity() < 0) || 
            (io.getAngle() <= ShoulderConstants.MIN_ANGLE && io.getVelocity() > 0)
        ) {
            io.stop();
        }
    }
}