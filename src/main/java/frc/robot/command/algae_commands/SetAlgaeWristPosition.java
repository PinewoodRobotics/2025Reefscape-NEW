// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.algae_commands;

import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetAlgaeWristPosition extends Command {
  
  private final AlgaeSubsystem m_algaeSubsystem;
  private final Rotation2d m_angle;
  private boolean m_waitUntilReach;

  public SetAlgaeWristPosition(AlgaeSubsystem algaeSubsystem, Rotation2d angle, boolean waitUntilReach) {
    m_algaeSubsystem = algaeSubsystem;
    m_angle = angle;
    m_waitUntilReach = waitUntilReach;

    addRequirements(algaeSubsystem);
  }

  @Override
  public void initialize() {
    m_algaeSubsystem.setWristPosition(m_angle);
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_algaeSubsystem.wristAtSetpoint() || !m_waitUntilReach;
  }
}
