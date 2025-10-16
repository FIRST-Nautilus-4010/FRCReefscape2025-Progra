package frc.robot.subsystems.elevators.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevators.ElevatorConstants;
import frc.robot.subsystems.elevators.ElevatorController;
import frc.robot.subsystems.elevators.ElevatorIO;
import frc.robot.subsystems.shoulder.ShoulderConstants;

public class MoveTo extends Command {
  private final double height;
  private final Supplier<Double> shoulderAngle;

  private final ElevatorController controller;
  private final ElevatorIO elevatorIO;

  public MoveTo(double height, Supplier<Double> shoulderAngle, ElevatorIO elevatorIO, SubsystemBase subsystem) {
    this.height = height;
    this.shoulderAngle = shoulderAngle;
    this.controller = new ElevatorController(elevatorIO.getLeaderMotor());
    this.elevatorIO = elevatorIO;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // Prevent elevator from moving if shoulder is not in a safe position
    if (
      (Math.toDegrees(Math.abs(shoulderAngle.get())) <= 2.0 || 
      ShoulderConstants.SAFETY_HEIGHT <= elevatorIO.getHeight()) &&
      height <= ElevatorConstants.MAX_HEIGHT &&
      height >= 0
    ) {
      controller.moveTo(height);
    } else {
      elevatorIO.stop();
    }
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(elevatorIO.getHeight() - height) <= 0.005) {
      System.out.println("terminando comando elevador");
      return true;
    }
    return false;
  }
}
