// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.BlankCommand;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.algae_commands.AlgaeEject;
import frc.robot.command.algae_commands.AlgaeIntake;
import frc.robot.command.algae_commands.HoldAlgae;
import frc.robot.command.alignment_commands.AlignReef;
import frc.robot.command.composites.ElevatorAndAlgae;
import frc.robot.command.composites.ElevatorAndCoral;
import frc.robot.command.composites.ManualScore;
import frc.robot.command.coral_commands.CoralIntake;
import frc.robot.command.coral_commands.HoldCoral;
import frc.robot.command.driving.DriveToPointOdometry;
import frc.robot.command.driving.OdomAssistedTagAlignment;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.command.finals.AutonAlignAndScore;
import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CompositeConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.config.TagConfig;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;
import frc.robot.util.controller.LogitechController;
import frc.robot.util.controller.OperatorPanel;

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
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
    m_leftFlightStick,
    m_rightFlightStick
  );
  OdomAssistedTagAlignment alignmentCommand;
  private final AHRSGyro m_gyro = new AHRSGyro(I2C.Port.kMXP);
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(
    m_gyro,
    new Communicator()
  );
  private final SwerveMoveTeleop m_moveCommand = new SwerveMoveTeleop(
    m_swerveDrive,
    m_flightModule
  );
  private final OdometrySubsystem m_odometrySubsystem;

  public RobotContainer() {
    m_odometrySubsystem = new OdometrySubsystem(m_swerveDrive, m_gyro);
    AprilTagSubsystem.launch(CameraConstants.kAprilTagPublicationTopic);

    setAlgaeCommands();
    setCoralCommands();
    setElevatorCommands();
    setSwerveCommands();
    setCompositeCommands();
    setAlignmentCommands();
  }

  public void setCompositeCommands() {
    m_leftFlightStick
      .A()
      .onTrue(
        new ElevatorAndCoral(
          m_elevatorSubsystem,
          m_coralSubsystem,
          CompositeConstants.kL4
        )
      );
    m_leftFlightStick
      .B()
      .onTrue(
        new ElevatorAndCoral(
          m_elevatorSubsystem,
          m_coralSubsystem,
          CompositeConstants.kL3
        )
      );
    m_leftFlightStick
      .X()
      .onTrue(
        new ElevatorAndCoral(
          m_elevatorSubsystem,
          m_coralSubsystem,
          CompositeConstants.kL2
        )
      );
    m_leftFlightStick
      .Y()
      .onTrue(
        new ElevatorAndAlgae(
          m_elevatorSubsystem,
          m_algaeSubsystem,
          CompositeConstants.kBottom
        )
      );
    m_rightFlightStick
      .trigger()
      .whileTrue(new ManualScore(m_coralSubsystem, m_elevatorSubsystem));
  }

  public void setElevatorCommands() {}

  public void setCoralCommands() {
    m_leftFlightStick.trigger().whileTrue(new CoralIntake(m_coralSubsystem));
    m_rightFlightStick
      .B5()
      .onTrue(
        m_coralSubsystem.runOnce(() -> m_coralSubsystem.calibrateWrist())
      );
    m_coralSubsystem.setDefaultCommand(new HoldCoral(m_coralSubsystem));
  }

  public void setAlgaeCommands() {
    m_rightFlightStick.B16().whileTrue(new AlgaeIntake(m_algaeSubsystem));
    m_rightFlightStick.B17().whileTrue(new AlgaeEject(m_algaeSubsystem));
    m_operatorPanel
      .blackButton()
      .onTrue(
        new ElevatorAndAlgae(
          m_elevatorSubsystem,
          m_algaeSubsystem,
          CompositeConstants.kHighAlgae
        )
      );
    m_operatorPanel
      .redButton()
      .onTrue(
        new ElevatorAndAlgae(
          m_elevatorSubsystem,
          m_algaeSubsystem,
          CompositeConstants.kMidAlgae
        )
      );
    m_operatorPanel
      .greenButton()
      .onTrue(
        new SetElevatorHeight(
          m_elevatorSubsystem,
          ElevatorConstants.kProcessorHeight,
          false
        )
      );
    m_algaeSubsystem.setDefaultCommand(new HoldAlgae(m_algaeSubsystem));
  }

  public void setAlignmentCommands() {
    alignmentCommand =
      new OdomAssistedTagAlignment(
        m_swerveDrive,
        m_odometrySubsystem,
        AlignmentConstants.poleLeft,
        AlignmentConstants.kDriveConfig,
        new TagConfig(
          100,
          AprilTagSubsystem.closestTagCurrently(
            AlignmentConstants.tagTimeThreshhold
          )
        ),
        AlignmentConstants.kSlowdownConfig,
        true,
        false,
        "TELEOP"
      );

    m_leftFlightStick
      .B7()
      .whileTrue(
        new Command() {
          @Override
          public void initialize() {
            alignmentCommand =
              new OdomAssistedTagAlignment(
                m_swerveDrive,
                m_odometrySubsystem,
                AlignmentConstants.poleLeft,
                AlignmentConstants.kDriveConfig,
                new TagConfig(
                  100,
                  AprilTagSubsystem.closestTagCurrently(
                    AlignmentConstants.tagTimeThreshhold
                  )
                ),
                AlignmentConstants.kSlowdownConfig,
                true,
                false,
                "TELEOP"
              );
          }
        }
      );

    m_leftFlightStick
      .B8()
      .whileTrue(
        new Command() {
          @Override
          public void initialize() {
            alignmentCommand =
              new OdomAssistedTagAlignment(
                m_swerveDrive,
                m_odometrySubsystem,
                AlignmentConstants.poleRight,
                AlignmentConstants.kDriveConfig,
                new TagConfig(
                  100,
                  AprilTagSubsystem.closestTagCurrently(
                    AlignmentConstants.tagTimeThreshhold
                  )
                ),
                AlignmentConstants.kSlowdownConfig,
                true,
                false,
                "TELEOP"
              );
          }
        }
      );

    m_leftFlightStick.B17().whileTrue(alignmentCommand);
  }

  public void setSwerveCommands() {
    m_swerveDrive.setDefaultCommand(m_moveCommand);
    m_rightFlightStick
      .B5()
      .onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro(180)));
    m_leftFlightStick
      .screenshare()
      .onTrue(new AlignReef(m_swerveDrive, m_moveCommand));
  }

  public Command getAutonomousCommand() {
    // return new MoveDirectionTimed(m_swerveDrive, -0.25, 0, 2000);
    if (
      m_operatorPanel.getRawButton(OperatorPanel.ButtonEnum.TOGGLEWHEELUP.value)
    ) {
      return new AutonAlignAndScore(
        m_swerveDrive,
        m_odometrySubsystem,
        m_elevatorSubsystem,
        m_coralSubsystem,
        CompositeConstants.kL3,
        AlignmentConstants.kDriveConfigAuton,
        AlignmentConstants.kSlowdownConfig,
        AlignmentConstants.poleLeft,
        AlignmentConstants.autonDriveForwardLeftSide,
        1000,
        500
      );
    } else if (
      m_operatorPanel.getRawButton(
        OperatorPanel.ButtonEnum.TOGGLEWHEELMIDUP.value
      )
    ) {
      return new MoveDirectionTimed(m_swerveDrive, -0.25, 0, 2000);
    } else if (
      m_operatorPanel.getRawButton(
        OperatorPanel.ButtonEnum.TOGGLEWHEELMIDDLE.value
      )
    ) {
      return new MoveDirectionTimed(m_swerveDrive, -1, 0, 15000);
    }

    return new BlankCommand();
  }

  public void onInit() {
    m_coralSubsystem.calibrateWrist();
    m_elevatorSubsystem.resetIAccum();
    m_algaeSubsystem.calibrateWrist();
  }
}
