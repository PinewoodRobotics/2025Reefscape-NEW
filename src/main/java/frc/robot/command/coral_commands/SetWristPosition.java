package frc.robot.command.coral_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class SetWristPosition extends Command {

  private CoralSubsystem m_coralSubsystem;
  private Rotation2d m_position;
  private boolean m_waitUntilReach;

  public SetWristPosition(CoralSubsystem coralSubsystem, Rotation2d position, boolean waitUntilReach) {
    m_coralSubsystem = coralSubsystem;
    m_position = position;
    m_waitUntilReach = waitUntilReach;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("||||||||||");
    m_coralSubsystem.setWristPosition(m_position);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("??????");
  }

  @Override
  public boolean isFinished() {
    return m_coralSubsystem.atSetpoint() || !m_waitUntilReach;
  }
}
