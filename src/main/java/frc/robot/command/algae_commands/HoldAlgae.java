package frc.robot.command.algae_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class HoldAlgae extends Command {

  private AlgaeSubsystem m_subsystem;

  public HoldAlgae(AlgaeSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_subsystem.hasAlgae()) {
      m_subsystem.runMotors(AlgaeConstants.kHoldSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
