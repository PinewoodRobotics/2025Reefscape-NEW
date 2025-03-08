// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.command.SwerveMoveAuto;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.PublicationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.position.Orientation;
import frc.robot.util.position.RobotPosition2d;
import proto.util.Position.Position2d;

public class RobotContainer {

  /*
  final FlightStick m_leftFlightStick = new FlightStick(
      OperatorConstants.kFlightPortLeft);
  
  final FlightStick m_rightFlightStick = new FlightStick(
      OperatorConstants.kFlightPortRight);
       */

  final XboxController controller = new XboxController(0);

  private final SwerveSubsystem m_swerveSubsystem;
  private final AHRSGyro m_gyroSubsystem;
  private final PathfindingSubsystem m_pathfindingSubsystem;
  private PublicationSubsystem m_publication;
  private LocalizationSubsystem m_localizationSubsystem;
  private LEDSubsystem led;

  public RobotContainer(Communicator communicator) {
    m_gyroSubsystem = new AHRSGyro(I2C.Port.kMXP);
    m_swerveSubsystem = new SwerveSubsystem(m_gyroSubsystem, communicator);
    m_localizationSubsystem = new LocalizationSubsystem(m_swerveSubsystem, m_gyroSubsystem);

    m_publication = new PublicationSubsystem();
    m_publication.addDataSubsystem(m_localizationSubsystem);

    m_pathfindingSubsystem = new PathfindingSubsystem(PathfindingConstants.mapFilePath,
        PathfindingConstants.rerunDistanceThreshhold, PathfindingConstants.maxNodesInRange,
        (float) (SwerveConstants.kDriveBaseWidth), (float) (SwerveConstants.kDriveBaseLength));
    m_pathfindingSubsystem.start();

    led = new LEDSubsystem(0, 60);

    LocalizationSubsystem.launch("pos-extrapolator/robot-position");
  }

  public void autonomousInit() {
    SwerveMoveAuto m_autoCommand = new SwerveMoveAuto(
        m_swerveSubsystem,
        led,
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
                    Orientation.FIELD));

            m_autoCommand.setIsDone(false);
          } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
          }
        });

    m_autoCommand.schedule();

    new JoystickButton(controller, XboxController.Button.kY.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              m_swerveSubsystem.resetGyro();
            }));
  }

  public void teleopInit() {
    m_swerveSubsystem.setDefaultCommand(
        new SwerveMoveTeleop(m_swerveSubsystem, controller));

    new JoystickButton(controller, XboxController.Button.kA.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              m_swerveSubsystem.resetGyro();
            }));
    new JoystickButton(controller, XboxController.Button.kY.value)
        .onTrue(
            m_swerveSubsystem.runOnce(() -> {
              m_swerveSubsystem.resetGyro();
            }));
  }
}
