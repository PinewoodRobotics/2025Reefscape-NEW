// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveToGoal;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PublicationSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommand;
import frc.robot.util.Autobahn;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightStick;
import java.util.concurrent.atomic.AtomicInteger;
import org.pwrup.util.Vec2;

public class RobotContainer {

  // private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  final FlightStick m_leftFlightStick = new FlightStick(
    OperatorConstants.kFlightPortLeft
  );

  final FlightStick m_rightFlightStick = new FlightStick(
    OperatorConstants.kFlightPortRight
  );

  private final Swerve m_swerveSubsystem;
  private final Gyro m_gyroSubsystem;
  final Autobahn autobahn;
  final Communicator communicator;
  private DriveToGoal m_driveToGoal;
  private PublicationSubsystem m_publication;

  public RobotContainer() {
    this.communicator = new Communicator();
    this.autobahn =
      Constants.GeneralDebugConstants.kEnableOffline
        ? null
        : new Autobahn(
          AutobahnConstants.kAutobahnHost,
          AutobahnConstants.kAutobahnPort
        );
    if (this.autobahn != null) {
      this.autobahn.begin()
        .thenRun(() ->
          System.out.println("Successfully connected to Autobahn server")
        )
        .exceptionally(ex -> {
          System.err.println(
            "Failed to connect to Autobahn server: " + ex.getMessage()
          );
          return null;
        });
    }
    Communicator.init(autobahn);

    m_gyroSubsystem = new Gyro(I2C.Port.kMXP);
    m_swerveSubsystem = new Swerve(m_gyroSubsystem, communicator);

    m_publication = new PublicationSubsystem();
    m_publication.addDataSubsystem(m_gyroSubsystem, m_swerveSubsystem);
  }

  public void autonomousInit() {
    /*m_driveToGoal =
      new DriveToGoal(
        m_swerveSubsystem,
        0.1,
        "pos-extrapolator/robot-position",
        "navigation/set_goal"
      );

    m_driveToGoal.setDefaultCommand(
      new RunCommand(
        () -> {
          m_driveToGoal.tick();
        },
        m_driveToGoal
      )
    );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
      .onTrue(
        m_driveToGoal.runOnce(() -> {
          m_driveToGoal.switchOnline();
        })
      );*/
  }

  public void teleopInit() {
    m_swerveSubsystem.setDefaultCommand(
      SwerveCommand.getManualRunCommand(
        m_swerveSubsystem,
        m_leftFlightStick,
        m_rightFlightStick
      )
    );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_swerveSubsystem.resetGyro();
        })
      );
  }
}
