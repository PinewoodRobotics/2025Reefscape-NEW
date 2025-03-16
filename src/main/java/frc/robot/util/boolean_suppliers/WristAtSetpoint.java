package frc.robot.util.boolean_suppliers;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.CoralSubsystem;

public class WristAtSetpoint implements BooleanSupplier {

  CoralSubsystem m_coralSubsystem;
  
  public WristAtSetpoint(
    CoralSubsystem coralSubsystem
  ) {
    m_coralSubsystem = coralSubsystem;
  }

  @Override
  public boolean getAsBoolean() {
    return m_coralSubsystem.atSetpoint();
  }
}
