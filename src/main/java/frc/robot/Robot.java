// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import autobahn.client.Address;
import autobahn.client.AutobahnClient;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.PiConstants;
import lombok.Getter;
import pwrup.frc.core.online.raspberrypi.OptionalAutobahn;

public class Robot extends LoggedRobot {
  @Getter
  private static OptionalAutobahn autobahnClient = new OptionalAutobahn();
  @Getter
  private static boolean onlineStatus = false;

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private Timer m_networkInitializeTimer = new Timer();

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.onRobotStart();

    m_networkInitializeTimer.reset();
    m_networkInitializeTimer.start();
    initializeNetwork();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.onPeriodic();

    Logger.recordOutput("Autobahn/Connected",
        autobahnClient.isConnected());
    Logger.recordOutput("Autobahn/FoundMainPi", onlineStatus);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.onAnyModeStart();
    m_robotContainer.onInit();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    m_robotContainer.onAnyModeStart();
    m_robotContainer.onInit();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private String readFromFile(File path) {
    try {
      return Files.readString(Paths.get(path.getAbsolutePath()));
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  private void initializeNetwork() {
    new Thread(() -> {
      PiConstants.network.initialize();
      onlineStatus = PiConstants.network.getMainPi() != null;

      if (onlineStatus) {
        // The main Pi is defined as the first one added to the network. In essence this
        // is here to create an addr to some pi to which the robot can connect. Without
        // going into too much detail, if the robot connects to one Pi, it starts to
        // receive data from anything running on the pi network (vision/etc.)
        var address = new Address(PiConstants.network.getMainPi().getHost(), PiConstants.network.getMainPi().getPort());
        var realClient = new AutobahnClient(address); // this is the pubsub server
        realClient.begin().join(); // this essentially attempts to connect to the pi specified in the
                                   // constructor.
        autobahnClient.setAutobahnClient(realClient);

        // Very important bit here:
        // The network has a -> shared config <- which must be sent to it on start. At
        // each pi in the network there runs a server listening to a port to which you
        // can send commands regarding the functionality of the pi (for example "start
        // [a, b, c]" or "stop [a, b, c]").
        // Anyway, these two commands 1) set the config on the pi (thereby updating the
        // pi config to your local typescript config) and 2) restart all the pi
        // processes (what this means is that the network, under the hood, sends 2
        // commands -- to stop all processes running on the pi and then to restart the
        // new selected processes)
        PiConstants.network.setConfig(readFromFile(PiConstants.configFilePath));
        boolean success = PiConstants.network.restartAllPis();
        if (!success) { // one of the exit codes is not successful in http req
          System.out.println("ERROR: Failed to restart Pis");
        }
      } else {
        System.out.println("WARNING: NO NETWORK INITIALIZED! SOME FEATURES MAY NOT BE AVAILABLE AT THIS TIME.");
      }
    }).start();
  }
}
