package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeAndHold extends Command { //NOT DONE

  private AlgaeSubsystem m_subsystem;

  private boolean m_hasAlgae;

  public AlgaeIntakeAndHold(AlgaeSubsystem subsystem) {
    this.m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_subsystem.runMotors(AlgaeConstants.kIntakeSpeed);

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
