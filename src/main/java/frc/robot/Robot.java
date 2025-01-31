// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlgaeIntake;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;
import frc.robot.util.controller.LogitechController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  final LogitechController m_controller = new LogitechController(0);
  final FlightModule m_flightModule = new FlightModule(
    2,
    3
  );

  /** Called once at the beginning of the robot program. */
  public Robot() {
    

    
    
  }

  public void setAlgaeCommands() {
    new JoystickButton(m_controller, LogitechController.ButtonEnum.A.value)
      .whileTrue(new AlgaeIntake(m_algaeSubsystem));
  }



  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
