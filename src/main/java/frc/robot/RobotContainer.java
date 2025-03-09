// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import org.pwrup.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.AlgaeIntake;
import frc.robot.command.CoralEject;
import frc.robot.command.CoralIntake;
import frc.robot.command.SetElevatorHeight;
import frc.robot.command.SetWristPosition;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.LogitechController;
import frc.robot.util.controller.FlightStick;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class RobotContainer {

  
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  final LogitechController m_controller = new LogitechController(0);
  final FlightModule m_flightModule = new FlightModule(2, 3);
  private final AHRSGyro m_gyro = new AHRSGyro(I2C.Port.kMXP);
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(m_gyro, new Communicator());

  public RobotContainer() {
    setAlgaeCommands();
    setCoralCommands();
    setElevatorCommands();
    setSwerveCommands();
  }

  public void setElevatorCommands() {
    new JoystickButton(m_controller, LogitechController.ButtonEnum.A.value)
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          Distance.ofRelativeUnits(1, Feet)
        )
      );
    new JoystickButton(m_controller, LogitechController.ButtonEnum.X.value)
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          Distance.ofRelativeUnits(3.5, Feet)
        )
      );
    new JoystickButton(m_controller, LogitechController.ButtonEnum.Y.value)
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          Distance.ofRelativeUnits(5, Feet)
        )
      );
  }

  public void setCoralCommands() {
    new JoystickButton(
      m_flightModule.leftFlightStick,
      FlightStick.ButtonEnum.B5.value
    )
      .whileTrue(
        new SetWristPosition(m_coralSubsystem, Rotation2d.fromDegrees(35))
      );
    new JoystickButton(
      m_flightModule.leftFlightStick,
      FlightStick.ButtonEnum.B6.value
    )
      .whileTrue(
        new SetWristPosition(m_coralSubsystem, Rotation2d.fromDegrees(-35))
      );
    new JoystickButton(
      m_flightModule.leftFlightStick,
      FlightStick.ButtonEnum.B7.value
    )
      .whileTrue(new CoralIntake(m_coralSubsystem));
    new JoystickButton(
      m_flightModule.leftFlightStick,
      FlightStick.ButtonEnum.B8.value
    )
      .whileTrue(new CoralEject(m_coralSubsystem));
  }

  public void setAlgaeCommands() {
    new JoystickButton(
      m_controller,
      LogitechController.ButtonEnum.RIGHTBUTTON.value
    )
      .whileTrue(new AlgaeIntake(m_algaeSubsystem));
    new JoystickButton(
      m_controller,
      LogitechController.ButtonEnum.LEFTBUTTON.value
    )
      .whileTrue(new AlgaeIntake(m_algaeSubsystem));
  }
  
  public void setSwerveCommands() {
    m_swerveDrive.setDefaultCommand(new SwerveMoveTeleop(m_swerveDrive, m_flightModule));
  }
}

// public void setAlgaeCommands() {
//   new JoystickButton(m_controller, LogitechController.ButtonEnum.RIGHTBUTTON.value)
//     .whileTrue(new AlgaeIntake(m_algaeSubsystem));
//   new JoystickButton(m_controller, LogitechController.ButtonEnum.LEFTBUTTON.value)
//     .whileTrue(new AlgaeIntake(m_algaeSubsystem));
// }
