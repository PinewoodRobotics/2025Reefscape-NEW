// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutobahnConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.RobotWheelMover;
import frc.robot.subsystems.DriveToGoal;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Autobahn;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightStick;
import java.util.concurrent.atomic.AtomicInteger;
import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;

public class RobotContainer {

  // private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

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

  public RobotContainer() {
    this.communicator = new Communicator();
    this.autobahn = null;
    /*this.autobahn =
      new Autobahn(
        AutobahnConstants.kAutobahnHost,
        AutobahnConstants.kAutobahnPort
      );*/

    m_gyroSubsystem = new Gyro(I2C.Port.kMXP, "robot/imu");
    m_swerveSubsystem =
      new Swerve(
        m_gyroSubsystem,
        "robot/odometry",
        "robot/update",
        communicator
      );
  }

  public void autonomousInit() {
    m_driveToGoal =
      new DriveToGoal(
        m_swerveSubsystem,
        0.1,
        "pos-extrapolator/robot-position",
        "navigation/set_goal"
      );

    m_driveToGoal.setDefaultCommand(
      new RunCommand(
        () -> {
          m_driveToGoal.tick();
        },
        m_driveToGoal
      )
    );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
      .onTrue(
        m_driveToGoal.runOnce(() -> {
          m_driveToGoal.switchOnline();
        })
      );
  }

  public void teleopInit() {
    // Add error handling for connection
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

    Communicator.init(autobahn);

    m_swerveSubsystem.setOdometryPosition(new Pose2d(0, 0, new Rotation2d(0)));

    AtomicInteger time = new AtomicInteger(0);
    m_swerveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> {
          m_swerveSubsystem.drive(
            new Vec2(
              CustomMath.deadband(
                m_rightFlightStick.getRawAxis(
                  FlightStick.AxisEnum.JOYSTICKY.value
                ) *
                -1,
                SwerveConstants.kXSpeedDeadband,
                SwerveConstants.kXSpeedMinValue
              ),
              CustomMath.deadband(
                m_rightFlightStick.getRawAxis(
                  FlightStick.AxisEnum.JOYSTICKX.value
                ),
                SwerveConstants.kYSpeedDeadband,
                SwerveConstants.kYSpeedMinValue
              )
            ),
            CustomMath.deadband(
              m_rightFlightStick.getRawAxis(
                FlightStick.AxisEnum.JOYSTICKROTATION.value
              ),
              SwerveConstants.kRotDeadband,
              SwerveConstants.kRotMinValue
            ),
            0.2
          );

          //m_swerveSubsystem.odometryTick();

          //if (time.get() > 10) {
            //m_swerveSubsystem.publishOdometry();
            //time.set(-1);
          //}

          time.incrementAndGet();
        },
        m_swerveSubsystem
      )
    );

    new JoystickButton(m_leftFlightStick, FlightStick.ButtonEnum.A.value)
      .onTrue(
        m_swerveSubsystem.runOnce(() -> {
          m_swerveSubsystem.resetGyro();
        })
      );
  }
}
