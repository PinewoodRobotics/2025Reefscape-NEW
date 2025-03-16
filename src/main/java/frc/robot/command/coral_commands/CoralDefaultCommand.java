package frc.robot.command.coral_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class CoralDefaultCommand extends Command {

  private CoralSubsystem m_subsystem;

  public CoralDefaultCommand(CoralSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setWristPosition(CoralConstants.kDefaultAngle);
  }

  @Override
  public void execute() {
    if (m_subsystem.hasCoral()) {
      m_subsystem.runIntake(CoralConstants.kHoldingSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
