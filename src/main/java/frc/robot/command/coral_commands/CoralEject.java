package frc.robot.command.coral_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class CoralEject extends Command {

  private CoralSubsystem m_coralSubsystem;
  private double m_startTime;

  /**
   * Runs the coral
   * @param subsystem coral
   */
  public CoralEject(CoralSubsystem subsystem) {
    this.m_coralSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_coralSubsystem.eject();

    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_coralSubsystem.eject();

    if (System.currentTimeMillis() - m_startTime > CoralConstants.kEjectTime) {
      CoralEject.this.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_coralSubsystem.stopIntake();
    m_coralSubsystem.setHoldingCoral(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
