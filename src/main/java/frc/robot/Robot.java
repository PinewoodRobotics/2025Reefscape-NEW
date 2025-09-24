// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import autobahn.client.Address;
import autobahn.client.AutobahnClient;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BotConstants;
import frc.robot.constants.PiConstants;
import lombok.Getter;

public class Robot extends LoggedRobot {
  @Getter
  private static AutobahnClient autobahnClient;

  private RobotContainer m_robotContainer;

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    switch (BotConstants.currentMode) {
      case REAL:
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    var address = new Address(PiConstants.network.getMainPi().getHost(), PiConstants.network.getMainPi().getPort());
    autobahnClient = new AutobahnClient(address);
    autobahnClient.begin().join();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.onRobotStart();

    PiConstants.network.setConfig(readFromFile(PiConstants.configFilePath));
    boolean success = PiConstants.network.restartAllPis();
    if (!success) {
      System.out.println("ERROR: Failed to restart Pis");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.onPeriodic();

    Logger.recordOutput("Autobahn/Connected", autobahnClient.isConnected());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.onInit();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
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
}
