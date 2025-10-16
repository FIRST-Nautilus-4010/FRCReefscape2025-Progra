package frc.robot.subsystems.endEffector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorIO;

public class Intake extends Command {
  private final EndEffectorIO io;

  public Intake(EndEffectorIO io, Subsystem... requirements) {
    this.io = io;

    addRequirements(requirements);
  }

  @Override
  public void execute() {
    io.set(-1);
  }

  @Override
  public void end(boolean interrupted) {
    io.stop();
  }

  @Override
  public boolean isFinished() {
    if (io.getCurrent() >= EndEffectorConstants.CURRENT_THRESHOLD) {
      System.out.println("termino end effector");
      return true;
    }
    return false;
  }
}
