package frc.robot.util.boolean_suppliers;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAtSetpoint implements BooleanSupplier {

  ElevatorSubsystem m_elevatorSubsystem;
  
  public ElevatorAtSetpoint(
    ElevatorSubsystem elevatorSubsystem
  ) {
    m_elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public boolean getAsBoolean() {
    return m_elevatorSubsystem.atTarget();
  }
}
