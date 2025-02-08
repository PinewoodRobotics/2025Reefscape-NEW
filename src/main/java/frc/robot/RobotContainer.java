// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SetWristPosition;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.LogitechController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class RobotContainer {
  
  // private final AlgaeSubsystem m_algaeSubsystem;
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  final LogitechController m_controller = new LogitechController(0);
  final FlightModule m_flightModule = new FlightModule(
    2,
    3
  );

  
  public RobotContainer() {
    // m_algaeSubsystem = new AlgaeSubsystem();

    // setAlgaeCommands();
    setCoralCommands();

    
    
  }
  
  public void setCoralCommands() {
    new JoystickButton(m_controller, LogitechController.ButtonEnum.B.value)
      .whileTrue(new SetWristPosition(m_coralSubsystem, Rotation2d.fromDegrees(90)));
    new JoystickButton(m_controller, LogitechController.ButtonEnum.Y.value)
      .whileTrue(new SetWristPosition(m_coralSubsystem, Rotation2d.fromDegrees(0)));
    new JoystickButton(m_controller, LogitechController.ButtonEnum.RIGHTTRIGGER.value)
      .whileTrue(new CoralIntake(m_coralSubsystem));
  }


  // public void setAlgaeCommands() {
  //   new JoystickButton(m_controller, LogitechController.ButtonEnum.RIGHTBUTTON.value)
  //     .whileTrue(new AlgaeIntake(m_algaeSubsystem));
  //   new JoystickButton(m_controller, LogitechController.ButtonEnum.LEFTBUTTON.value)
  //     .whileTrue(new AlgaeIntake(m_algaeSubsystem));
  // }

}
