package frc.robot.subsystems.shoulder.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.shoulder.ShoulderController;
import frc.robot.subsystems.shoulder.ShoulderIO;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;

public class PutCmd extends Command {
  private final ShoulderController controller;
  private final ShoulderIO io;
  private final ShoulderSubsystem shoulderSubsystem;

  private boolean finish = false;
  
  private double desPos;

  public PutCmd(ShoulderSubsystem shoulderSubsystem, ShoulderController controller, ShoulderIO io) {
    this.controller = controller;
    this.io = io;
    this.shoulderSubsystem = shoulderSubsystem;

    this.setName("ShoulderPutCmd");

    addRequirements(shoulderSubsystem);
  }
  @Override
  public void initialize() {
    desPos = io.getAngle() + ShoulderConstants.PUT;

    if (
      !(
        shoulderSubsystem.getCurrentCommand().getName().equals("ShoulderIntakeCmd") || 
        shoulderSubsystem.getCurrentCommand().getName().equals("ShoulderL1Cmd") ||
        shoulderSubsystem.getCurrentCommand().getName().equals("ShoulderPutCmd")
      )
    ) {
      controller.goTo(desPos);
    } else {
      finish = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(io.getAngle() - desPos) < 0.1) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return finish;
  }
}
