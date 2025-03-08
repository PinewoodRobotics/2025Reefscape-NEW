package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeight extends Command {

  private ElevatorSubsystem m_subsystem;
  private Distance m_position;

  public SetElevatorHeight(ElevatorSubsystem subsystem, Distance height) {
    this.m_subsystem = subsystem;
    this.m_position = height;

    addRequirements(subsystem); //two commands that require the same subsystem are not allowed to run at the same time
  }

  @Override
  public void initialize() {
    System.out.println("initializing SetElevatorHeight Command");
    m_subsystem.setHeight(m_position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
