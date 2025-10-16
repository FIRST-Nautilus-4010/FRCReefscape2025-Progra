// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.shoulder.ShoulderController;
import frc.robot.subsystems.shoulder.ShoulderIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTo extends Command {
  private final double angle;
  private final Supplier<Double> pivotHeight;

  private final ShoulderController controller;
  private final ShoulderIO shoulderIO;

  /** Creates a new RotateTo. */
  public RotateTo(double angle, Supplier<Double> pivotHeight, ShoulderIO shoulderIO, SubsystemBase subsystem) {
    this.angle = angle;
    this.pivotHeight = pivotHeight;

    this.controller = new ShoulderController(shoulderIO);
    this.shoulderIO = shoulderIO;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    if (
      pivotHeight.get() >= ShoulderConstants.SAFETY_HEIGHT && 
      angle <= ShoulderConstants.MAX_ANGLE && 
      angle >= ShoulderConstants.MIN_ANGLE
    ) {
      controller.rotateTo(angle);
    } else {
      shoulderIO.stop();
    }
  }

  @Override
  public boolean isFinished() {
    double realAngle = shoulderIO.getAngle();
    if (Double.isInfinite(realAngle)) {
      realAngle = 0;
      System.out.println("el angulo es infinito");
    }

    if (Math.abs(realAngle - angle) <= 0.4) {
      System.out.println("terminando comando shoulder");
      return true;
    }
    System.out.println("angle diference: " + (realAngle - angle));
    return false;
  }
}
