package frc.robot.util.boolean_suppliers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAboveHeight implements BooleanSupplier {
  
  private ElevatorSubsystem m_elevatorSubsystem;
  private Distance m_height;

  public ElevatorAboveHeight(
    ElevatorSubsystem elevatorSubsystem,
    Distance height
  ) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_height = height;
  }

  @Override
  public boolean getAsBoolean() {
    return m_elevatorSubsystem.getAverageHeight().gt(m_height);
  }
}
