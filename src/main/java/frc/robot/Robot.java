// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.util.Communicator;
import frc.robot.util.online.Address;
import frc.robot.util.online.Autobahn;
import frc.robot.util.online.RaspberryPi;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private Autobahn autobahn;
  private Communicator communicator;
  private File configFilePath = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/config.json");

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    this.communicator = new Communicator();
    this.autobahn = Constants.GeneralDebugConstants.kEnableOffline
        ? null
        : new Autobahn(new Address[] {
            AutobahnConstants.tripoli.address
        });
    if (this.autobahn != null) {
      this.autobahn.begin()
          .thenRun(() -> {
            System.out.println(
                "Successfully connected to Autobahn server. Sending pi initialization commands...");

            for (RaspberryPi pi : AutobahnConstants.all) {
              pi.initialize(configFilePath);
            }
          })
          .exceptionally(ex -> {
            System.err.println(
                "Failed to connect to Autobahn server: " + ex.getMessage());
            return null;
          });
    }

    Communicator.init(autobahn);

    m_robotContainer = new RobotContainer(communicator);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override

  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.teleopInit();
  }
}
