// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.IntakeController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetState extends Command {
  private final double vel;
  private final double pos;
  private final IntakeController controller;

  Supplier<Double> ampFunc;

  public SetState(double vel, double pos, IntakeController controller, Supplier<Double> ampFunc, Subsystem subsystem) {
    this.vel = vel;
    this.pos = pos;
    this.controller = controller;

    this.ampFunc = ampFunc;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    controller.setVelocity(vel);
    controller.goTo(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setVelocity(vel);
    controller.goTo(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ampFunc.get() > 70) {
      System.out.println("terminando comando intake");
      return true;
    }
    return false;
  }
}
