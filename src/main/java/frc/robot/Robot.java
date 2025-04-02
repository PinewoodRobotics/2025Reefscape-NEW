// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutobahnConstants;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.online.Autobahn;
import frc.robot.util.online.RaspberryPi;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Autobahn autobahn;

  @Override
  public void robotInit() {
    autobahn = new Autobahn(AutobahnConstants.tripoli.address);

    if (!AutobahnConstants.kEnableOffline) {
      this.autobahn.begin()
        .thenRun(() -> {
          System.out.println(
            "Successfully connected to Autobahn server. Sending pi initialization commands..."
          );

          for (RaspberryPi pi : AutobahnConstants.all) {
            pi.initialize(AutobahnConstants.configFilePath);
          }
        })
        .exceptionally(ex -> {
          System.err.println(
            "Failed to connect to Autobahn server: " + ex.getMessage()
          );
          return null;
        });

      Communicator.init(autobahn);
      AprilTagSubsystem.launch(CameraConstants.kAprilTagPublicationTopic);
    }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.onInit();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.onInit();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
