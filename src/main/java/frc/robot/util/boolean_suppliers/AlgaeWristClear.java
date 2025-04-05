package frc.robot.util.boolean_suppliers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeWristClear implements BooleanSupplier {

  private ElevatorSubsystem m_elevatorSubsystem;
  private Distance m_setHeight;

  public AlgaeWristClear(
      ElevatorSubsystem elevatorSubsystem,
      Distance setHeight) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_setHeight = setHeight;
  }

  @Override
  public boolean getAsBoolean() {
    return m_elevatorSubsystem.getAverageHeight().gt(ElevatorConstants.kReefClearanceHeight)
        || m_setHeight.lt(ElevatorConstants.kReefClearanceHeight);
  }
}
