package frc.robot.command.coral_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class CoralIntake extends Command {

  private CoralSubsystem m_subsystem;

  private double m_startTime;

  public CoralIntake(CoralSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setWristPosition(CoralConstants.kIntakeAngle);
    m_subsystem.intake();

    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_subsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
    m_subsystem.setHoldingCoral(true);
    m_subsystem.setWristPosition(CoralConstants.kDefaultAngle);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime > 500)
    && (m_subsystem.getIntakeCurrent() > CoralConstants.kIntakeCurrentThreshold);
  }
}
