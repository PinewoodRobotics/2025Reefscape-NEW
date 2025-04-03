// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.OldConstants.OperatorConstants;
import frc.robot.command.OdomAssistedTagAlignment;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import frc.robot.util.config.TagConfig;
import frc.robot.util.controller.FlightStick;
import frc.robot.util.position.RobotPosition2d;
import proto.util.Position.Position2d;

public class RobotContainer {

  final FlightStick m_leftFlightStick = new FlightStick(
    OperatorConstants.kFlightPortLeft
  );

  final FlightStick m_rightFlightStick = new FlightStick(
    OperatorConstants.kFlightPortRight
  );

  final XboxController m_controller = new XboxController(3);

  private final SwerveSubsystem m_swerveSubsystem;
  private final AHRSGyro m_gyroSubsystem;
  private final OdometrySubsystem m_odometrySubsystem;

  public RobotContainer(Communicator communicator) {
    m_gyroSubsystem = new AHRSGyro(I2C.Port.kMXP);
    m_swerveSubsystem = new SwerveSubsystem(m_gyroSubsystem, communicator);
    m_odometrySubsystem =
      new OdometrySubsystem(m_swerveSubsystem, m_gyroSubsystem);

    AprilTagSubsystem.launch("apriltag/tag");
  }

  public void autonomousInit() {}

  public void teleopInit() {
    m_swerveSubsystem.setDefaultCommand(
      new SwerveMoveTeleop(m_swerveSubsystem, m_leftFlightStick)
    );

    new JoystickButton(m_controller, XboxController.Button.kA.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_swerveSubsystem.resetGyro();
        })
      );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.B17.value)
      .whileTrue(
        new OdomAssistedTagAlignment(
          m_swerveSubsystem,
          m_odometrySubsystem,
          PathfindingConstants.pole1,
          new DriveConfig(0.03, 2, 0.2, 0.2),
          new TagConfig(100, 9),
          new SlowdownConfig(
            SwerveConstants.secondTierDistance,
            SwerveConstants.thirdTierDistance,
            SwerveConstants.firstTierMaxSpeedMultiplier,
            SwerveConstants.secondTierMaxSpeedMultiplier,
            SwerveConstants.thirdTierMaxSpeedMultiplier
          ),
          true,
          false
        )
      );

    new JoystickButton(m_rightFlightStick, FlightStick.ButtonEnum.B6.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_gyroSubsystem.reset();
        })
      );

    new JoystickButton(m_rightFlightStick, FlightStick.ButtonEnum.B17.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_odometrySubsystem.setOdometryPosition(new Pose2d());
        })
      );
  }
}
