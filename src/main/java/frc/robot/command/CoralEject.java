package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class CoralEject extends Command {

  private CoralSubsystem m_subsystem;

  public CoralEject(CoralSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.runIntake(-CoralConstants.kIntakeSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
    m_subsystem.setHoldingCoral(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
