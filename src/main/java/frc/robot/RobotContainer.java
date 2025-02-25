// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.command.SwerveMoveAuto;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.PublicationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Address;
import frc.robot.util.Autobahn;
import frc.robot.util.Communicator;
import frc.robot.util.controller.FlightStick;
import frc.robot.util.position.RobotPosition2d;
import frc.robot.util.position.RobotPositionType;
import proto.util.Position.Position2d;

public class RobotContainer {

  final FlightStick m_leftFlightStick = new FlightStick(
      OperatorConstants.kFlightPortLeft);

  final FlightStick m_rightFlightStick = new FlightStick(
      OperatorConstants.kFlightPortRight);

  private final SwerveSubsystem m_swerveSubsystem;
  private final AHRSGyro m_gyroSubsystem;
  final Autobahn autobahn;
  final Communicator communicator;
  private PublicationSubsystem m_publication;
  private LocalizationSubsystem m_localizationSubsystem;

  public RobotContainer() {
    this.communicator = new Communicator();
    this.autobahn = Constants.GeneralDebugConstants.kEnableOffline
        ? null
        : new Autobahn(
            new Address(
                AutobahnConstants.kAutobahnHost,
                AutobahnConstants.kAutobahnPort));
    if (this.autobahn != null) {
      this.autobahn.begin()
          .thenRun(() -> System.out.println("Successfully connected to Autobahn server"))
          .exceptionally(ex -> {
            System.err.println(
                "Failed to connect to Autobahn server: " + ex.getMessage());
            return null;
          });
    }
    Communicator.init(autobahn);

    m_gyroSubsystem = new AHRSGyro(I2C.Port.kMXP);
    m_swerveSubsystem = new SwerveSubsystem(m_gyroSubsystem, communicator);
    m_localizationSubsystem = new LocalizationSubsystem(m_swerveSubsystem, m_gyroSubsystem);

    m_publication = new PublicationSubsystem();
    m_publication.addDataSubsystem(m_localizationSubsystem);
    LocalizationSubsystem.launch("pos-extrapolator/robot-position");
  }

  public void autonomousInit() {
    var m_autoCommand = new SwerveMoveAuto(
        m_swerveSubsystem,
        new RobotPosition2d(),
        true);

    Communicator.subscribeAutobahn(
        "auto/command",
        data -> {
          Position2d position;
          try {
            position = Position2d.parseFrom(data);
            m_autoCommand.setFinalPosition(
                new RobotPosition2d(
                    (double) position.getPosition().getX(),
                    (double) position.getPosition().getY(),
                    new Rotation2d(
                        position.getDirection().getX(),
                        position.getDirection().getY()),
                    RobotPositionType.GLOBAL));

            m_autoCommand.setIsDone(false);
          } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
          }
        });

    m_autoCommand.schedule();
  }

  public void teleopInit() {
    m_swerveSubsystem.setDefaultCommand(
        new SwerveMoveTeleop(m_swerveSubsystem, m_rightFlightStick));

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              m_swerveSubsystem.resetGyro();
            }));
  }
}
