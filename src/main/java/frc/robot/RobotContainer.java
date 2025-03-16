// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.algae_commands.AlgaeEject;
import frc.robot.command.algae_commands.AlgaeIntake;
import frc.robot.command.composites.ElevatorAndCoral;
import frc.robot.command.coral_commands.CoralEject;
import frc.robot.command.coral_commands.CoralIntake;
import frc.robot.command.coral_commands.HoldCoral;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.CompositeConstants;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
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
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(m_leftFlightStick, m_rightFlightStick);
  private final AHRSGyro m_gyro = new AHRSGyro(I2C.Port.kMXP);
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(m_gyro, new Communicator());

  public RobotContainer() {
    setAlgaeCommands();
    setCoralCommands();
    setElevatorCommands();
    setSwerveCommands();
  }

  public void setCompositeCommands() {
    m_leftFlightStick.B5()
      .onTrue(
        new ElevatorAndCoral(m_elevatorSubsystem, m_coralSubsystem, CompositeConstants.kL4)
      );
    m_leftFlightStick.B6()
      .onTrue(
        new ElevatorAndCoral(m_elevatorSubsystem, m_coralSubsystem, CompositeConstants.kL3)
      );
    m_leftFlightStick.B7()
      .onTrue(
        new ElevatorAndCoral(m_elevatorSubsystem, m_coralSubsystem, CompositeConstants.kL2)
      );
  }

  public void setElevatorCommands() {
    m_rightFlightStick.Y()
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          ElevatorConstants.kMinHeight,
          false
        )
      );
    m_rightFlightStick.X()
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          ElevatorConstants.kProcessorHeight,
          false
        )
      );
    m_rightFlightStick.B()
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          ElevatorConstants.kMidAlgaeHeight,
          false
        )
      );
    m_rightFlightStick.A()
      .whileTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          ElevatorConstants.kHighAlgaeHeight,
          false
        )
      );
    
  }

  public void setCoralCommands() {
    m_coralSubsystem.setDefaultCommand(
      new HoldCoral(m_coralSubsystem)
    );
    m_leftFlightStick.rightSliderUp()
      .whileTrue(
        new SetWristPosition(m_coralSubsystem, CoralConstants.kIntakeAngle, false)
      );
    m_leftFlightStick.rightSliderUp()
      .whileTrue(
        new CoralIntake(m_coralSubsystem)
      );
    m_leftFlightStick.rightSliderDown()
      .whileTrue(
        new CoralEject(m_coralSubsystem)
      );
  }

  public void setAlgaeCommands() {
    m_rightFlightStick.B16()
      .whileTrue(new AlgaeIntake(m_algaeSubsystem));
    m_rightFlightStick.B17()
      .whileTrue(new AlgaeEject(m_algaeSubsystem));
  }
  
  public void setSwerveCommands() {
    m_swerveDrive.setDefaultCommand(new SwerveMoveTeleop(m_swerveDrive, m_flightModule));
    m_rightFlightStick.B5()
      .onTrue(
        m_swerveDrive.runOnce(
          () -> m_swerveDrive.resetGyro()
        )
      );
  }
}

