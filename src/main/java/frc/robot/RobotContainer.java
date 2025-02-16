// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.PublicationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveCommand;
import frc.robot.util.Autobahn;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightStick;
import java.util.concurrent.atomic.AtomicInteger;
import org.pwrup.util.Vec2;
import proto.SetPositionOuterClass.SetPosition;

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
            AutobahnConstants.kAutobahnHost,
            AutobahnConstants.kAutobahnPort);
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
    m_publication.addDataSubsystem(m_localizationSubsystem, m_gyroSubsystem);
  }

  public void autonomousInit() {
  }

  public void teleopInit() {
    /*
    m_swerveSubsystem.setDefaultCommand(
        SwerveCommand.getManualRunCommand(
            m_swerveSubsystem,
            m_leftFlightStick,
            m_rightFlightStick));
    
    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              m_swerveSubsystem.resetGyro();
            }));
    
    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.B.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              SwerveCommand
                  .getCalibRunCommand(m_swerveSubsystem, m_gyroSubsystem)
                  .run();
            }))
        .onFalse(m_swerveSubsystem.runOnce(SwerveCommand::unsubscribeCalib));
         */
  }
}
