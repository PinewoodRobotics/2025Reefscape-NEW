package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualScore extends Command {

  CoralSubsystem m_coralSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  
  public ManualScore(
    CoralSubsystem coralSubsystem,
    ElevatorSubsystem elevatorSubsystem
  ) {
    m_coralSubsystem = coralSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    
    addRequirements(coralSubsystem, elevatorSubsystem);
  }

  @Override
  public void initialize() {
    m_coralSubsystem.eject();
  }

  @Override
  public void execute() {
    m_coralSubsystem.eject();
  }

  @Override
  public void end(boolean interrupted) {
    m_coralSubsystem.stopIntake();
    m_coralSubsystem.setHoldingCoral(false);
    m_coralSubsystem.setWristPosition(CoralConstants.kDefaultAngle);
    m_elevatorSubsystem.setHeight(ElevatorConstants.kDefaultHeight);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
