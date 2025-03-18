package frc.robot.command.coral_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class HoldCoral extends Command {

  private CoralSubsystem m_subsystem;

  public HoldCoral(CoralSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_subsystem.hasCoral() && m_subsystem.getSetpoint().getDegrees() < 0) {
      m_subsystem.runIntake(CoralConstants.kHoldingSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
