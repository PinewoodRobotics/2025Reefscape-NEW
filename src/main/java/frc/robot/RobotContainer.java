// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PublicationSubsystem;
import frc.robot.subsystems.swerve.DriveToGoal;
import frc.robot.subsystems.swerve.Swerve;
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
    m_publication.addDataSubsystem(m_swerveSubsystem); // m_gyroSubsystem
  }

  public void autonomousInit() {}

  public void teleopInit() {
    /*
    Communicator.subscribeAutobahn(
      "pos-extrapolator/set-position",
      data -> {
        try {
          var pos = SetPosition.parseFrom(data);
          switch (pos.getSetType()) {
            case IMU:
              m_gyroSubsystem.setPositionAdjustment(
                pos.getNewPosition().getPosition().getX(),
                pos.getNewPosition().getPosition().getY(),
                0
              );
              break;
            case ODOMETRY:
              m_swerveSubsystem.resetOdometryPosition(
                new Pose2d(
                  pos.getNewPosition().getPosition().getX(),
                  pos.getNewPosition().getPosition().getY(),
                  new Rotation2d(
                    pos.getNewPosition().getDirection().getX(),
                    pos.getNewPosition().getDirection().getY()
                  )
                )
              );
              break;
            default:
              break;
          }
        } catch (InvalidProtocolBufferException e) {
          e.printStackTrace();
        }
      }
    );
     */

    m_swerveSubsystem.setDefaultCommand(
      SwerveCommand.getManualRunCommand(
        m_swerveSubsystem,
        m_leftFlightStick,
        m_rightFlightStick
      )
    );

    m_publication.setDefaultCommand(
      new RunCommand(
        () -> {
          m_publication.tick();
        },
        m_publication
      )
    );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_swerveSubsystem.resetGyro();
        })
      );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.B.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          SwerveCommand
            .getCalibRunCommand(m_swerveSubsystem, m_gyroSubsystem)
            .run();
        })
      )
      .onFalse(m_swerveSubsystem.runOnce(SwerveCommand::unsubscribeCalib));
  }
}
