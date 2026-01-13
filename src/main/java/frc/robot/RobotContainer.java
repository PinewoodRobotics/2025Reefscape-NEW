// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.algae_commands.AlgaeEject;
import frc.robot.command.algae_commands.AlgaeIntake;
import frc.robot.command.algae_commands.HoldAlgae;
import frc.robot.command.auto_driving_commands.driving.ExecuteTrajectory;
import frc.robot.command.auto_driving_commands.misc.Pathfind;
import frc.robot.command.composites.ElevatorAndAlgae;
import frc.robot.command.composites.ElevatorAndCoral;
import frc.robot.command.composites.ManualScore;
import frc.robot.command.coral_commands.CoralIntake;
import frc.robot.command.coral_commands.HoldCoral;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.command.finals.AutonAlignAndScore;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.BotConstants;
import frc.robot.constants.BotConstants.RobotVariant;
import frc.robot.constants.CompositeConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.hardware.PigeonGyro;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AlignmentPoints;
import frc.robot.util.PathPlannerSetup;
import frc.robot.util.RPC;
import frc.robot.util.config.AlgaeElevatorConfig;
import proto.pathfind.Pathfind.Algorithm;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.LogitechController;
import pwrup.frc.core.controller.OperatorPanel;
import pwrup.frc.core.online.PublicationSubsystem;
import pwrup.frc.core.online.raspberrypi.PrintPiLogs;

public class RobotContainer {

  final LogitechController m_controller = new LogitechController(0);
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);
  SwerveMoveTeleop m_moveCommand;
  private static final String kPathPlannerAutoName = "MainAuto";

  public RobotContainer() {
    if (BotConstants.robotType == RobotVariant.ABOT) {
      OdometrySubsystem.GetInstance();
      CoralSubsystem.GetInstance();
      SwerveSubsystem.GetInstance();
      AlgaeSubsystem.GetInstance();
      CameraSubsystem.GetInstance();
      ElevatorSubsystem.GetInstance();
      PigeonGyro.GetInstance();

      setAlgaeCommands();
      setCoralCommands();
      setElevatorCommands();
      setCompositeCommands();

      PathPlannerSetup.configure();
    }

    AlignmentPoints.setPoints(AlignmentConstants.POINTS);

    setSwerveCommands();

    GlobalPosition.GetInstance();
    AHRSGyro.GetInstance().reset();
    SwerveSubsystem.GetInstance().resetGyro();

    PrintPiLogs.ToSystemOut(Robot.getAutobahnClient(), "pi-technical-log");
    RPC.SetClient(Robot.getAutobahnClient());
    PublicationSubsystem.GetInstance(Robot.getAutobahnClient());
  }

  public void setCompositeCommands() {
    m_leftFlightStick
        .A()
        .onTrue(
            new ElevatorAndCoral(
                ElevatorSubsystem.GetInstance(),
                CoralSubsystem.GetInstance(),
                CompositeConstants.kL4));
    m_leftFlightStick
        .B()
        .onTrue(
            new ElevatorAndCoral(
                ElevatorSubsystem.GetInstance(),
                CoralSubsystem.GetInstance(),
                CompositeConstants.kL3));
    m_leftFlightStick
        .X()
        .onTrue(
            new ElevatorAndCoral(
                ElevatorSubsystem.GetInstance(),
                CoralSubsystem.GetInstance(),
                CompositeConstants.kL2));
    m_leftFlightStick
        .Y()
        .onTrue(
            new ElevatorAndAlgae(
                ElevatorSubsystem.GetInstance(),
                AlgaeSubsystem.GetInstance(),
                CompositeConstants.kBottom));
    m_rightFlightStick
        .trigger()
        .whileTrue(new ManualScore(CoralSubsystem.GetInstance(),
            ElevatorSubsystem.GetInstance()));
  }

  public void setElevatorCommands() {
  }

  public void setCoralCommands() {
    CoralSubsystem coralSubsystem = CoralSubsystem.GetInstance();
    m_leftFlightStick.trigger().whileTrue(new CoralIntake(coralSubsystem));
    m_rightFlightStick
        .B5()
        .onTrue(
            coralSubsystem.runOnce(() -> coralSubsystem.calibrateWrist()));
    coralSubsystem.setDefaultCommand(new HoldCoral(coralSubsystem));
  }

  public void setAlgaeCommands() {
    AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.GetInstance();
    ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.GetInstance();
    m_rightFlightStick.B16().whileTrue(new AlgaeIntake(algaeSubsystem));
    m_rightFlightStick.B17().whileTrue(new AlgaeEject(algaeSubsystem));
    m_operatorPanel
        .blackButton()
        .onTrue(
            new ElevatorAndAlgae(
                elevatorSubsystem,
                algaeSubsystem,
                CompositeConstants.kHighAlgae));
    m_operatorPanel
        .redButton()
        .onTrue(
            new ElevatorAndAlgae(
                elevatorSubsystem,
                algaeSubsystem,
                CompositeConstants.kMidAlgae));
    m_operatorPanel
        .greenButton()
        .onTrue(
            new SetElevatorHeight(
                elevatorSubsystem,
                ElevatorConstants.kProcessorHeight,
                false));
    m_operatorPanel
        .metalSwitchDown()
        .onTrue(
            new ElevatorAndAlgae(
                elevatorSubsystem,
                algaeSubsystem,
                new AlgaeElevatorConfig(
                    ElevatorConstants.kRestingHeight,
                    AlgaeConstants.kIntakeAngle)));

    algaeSubsystem.setDefaultCommand(new HoldAlgae(algaeSubsystem));
  }

  public void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

    this.m_moveCommand = new SwerveMoveTeleop(swerveSubsystem, m_flightModule,
        new Supplier<Optional<Pose2d>>() {

          @Override
          public Optional<Pose2d> get() {
            if (m_moveCommand.isHeadingControl()) {
              return Optional.of(SwerveConstants.INSTANCE.headingControl);
            }

            return Optional.empty();
          }
        });

    swerveSubsystem.setDefaultCommand(m_moveCommand);
    m_rightFlightStick
        .B6()
        .onTrue(swerveSubsystem.runOnce(() -> {
          m_moveCommand.toggleHeadingControl();
        }));

    m_rightFlightStick
        .B5()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetGyro(0);
        })); /* was 180 */

    m_rightFlightStick.B8()
        .onTrue(SwerveSubsystem.GetInstance().runOnce(() -> {
          if (!Robot.isOnlineStatus()) {
            System.out.println("Not online, skipping pathfinding");
            return;
          }

          System.out.println("Pathfinding to (15, 20)");
          Pathfind pathfind = new Pathfind(new Pose2d(15, 20, new Rotation2d(0)), Algorithm.ASTAR, true);
          pathfind.initialize();
          try {
            while (!pathfind.isFinished()) {
              Thread.onSpinWait();
            }
            var result = pathfind.getResult();
            System.out.println("PathfindResult: " + result);
            var translationList = pathfind.getResultAsTranslation2dList();
            System.out.println("Result as Translation2d List: " + translationList);
          } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error pathfinding: " + e.getMessage());
          }
        }));

    m_leftFlightStick.B5().whileTrue(OdometrySubsystem.GetInstance()
        .runOnce(() -> OdometrySubsystem.GetInstance().setOdometryPosition(GlobalPosition.Get())));

    m_leftFlightStick
        .B16()
        .whileTrue(
            new ExecuteTrajectory(AlignmentPoints.getPoint("Coral:Closest:Left").get()));

    m_leftFlightStick
        .B17()
        .whileTrue(
            new ExecuteTrajectory(AlignmentPoints.getPoint("Coral:Closest:Right").get()));
  }

  public void onInit() {
    if (BotConstants.robotType == RobotVariant.ABOT) {
      CoralSubsystem.GetInstance().calibrateWrist();
      ElevatorSubsystem.GetInstance().resetIAccum();
      AlgaeSubsystem.GetInstance().calibrateWrist();
    }
  }

  public void onRobotStart() {
  }

  public void onPeriodic() {
  }

  public Command getAutonomousCommand() {
    if (m_operatorPanel.getRawButton(5)) {
      return new AutonAlignAndScore(
          SwerveSubsystem.GetInstance(),
          OdometrySubsystem.GetInstance(),
          ElevatorSubsystem.GetInstance(),
          CoralSubsystem.GetInstance(),
          CompositeConstants.kL4,
          AlignmentConstants.Coral.left,
          2200);
    } else if (m_operatorPanel.getRawButton(6)) {
      return new AutonAlignAndScore(
          SwerveSubsystem.GetInstance(),
          OdometrySubsystem.GetInstance(),
          ElevatorSubsystem.GetInstance(),
          CoralSubsystem.GetInstance(),
          CompositeConstants.kL4,
          AlignmentConstants.Coral.right,
          2200);
    } else if (m_operatorPanel.getRawButton(7)) {
      return new MoveDirectionTimed(SwerveSubsystem.GetInstance(), -0.3, 0, 2000);
      // return new PathPlannerAuto(kPathPlannerAutoName);
    }

    return null;
  }

  public void onAnyModeStart() {
    var position = GlobalPosition.Get();
    if (position == null) {
      System.out.println("ERROR: THERE IS NO GLOBAL POSITION ON START!");
      return;
    }

    AHRSGyro.GetInstance().setAngleAdjustment(position.getRotation().getDegrees());
    OdometrySubsystem.GetInstance().setOdometryPosition(position);
    PublicationSubsystem.addDataClasses(OdometrySubsystem.GetInstance(),
        AHRSGyro.GetInstance());
  }
}
