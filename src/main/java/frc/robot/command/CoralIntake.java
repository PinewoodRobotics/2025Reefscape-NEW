package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class CoralIntake extends Command {

  private CoralSubsystem m_subsystem;

  public CoralIntake(CoralSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.runIntake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
    m_subsystem.setHoldingCoral(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
