package frc.robot.command;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import lombok.Getter;
import lombok.Setter;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;
  private final Supplier<Optional<Pose2d>> headingControl;

  @Getter
  @Setter
  private boolean isHeadingControl = false;
  private static final double MAX_OMEGA = 2 * Math.PI;
  private static final double MAX_ALPHA = 2;
  private final ProfiledPIDController headingController = new ProfiledPIDController(
      3,
      SwerveConstants.INSTANCE.kHeadingI,
      SwerveConstants.INSTANCE.kHeadingD,
      new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA));

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller,
      Supplier<Optional<Pose2d>> headingControl) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    this.headingControl = headingControl;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerveSubsystem);
  }

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller) {
    this(swerveSubsystem, controller, new Supplier<Optional<Pose2d>>() {

      @Override
      public Optional<Pose2d> get() {
        return Optional.empty();
      }
    });
  }

  @Override
  public void initialize() {
    var globalPose = GlobalPosition.Get();
    if (headingControl != null && globalPose != null) {
      // Reset controller with current heading when heading control is activated
      headingController.reset(globalPose.getRotation().getRadians());
    }
  }

  @Override
  public void execute() {
    final var c = SwerveConstants.INSTANCE;
    double r = CustomMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value) * -1,
        c.kRotDeadband,
        c.kRotMinValue);

    double x = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKY.value) * -1,
        c.kXSpeedDeadband,
        c.kXSpeedMinValue);

    double y = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKX.value) *
            -1,
        c.kYSpeedDeadband,
        c.kYSpeedMinValue);

    if (headingControl.get().isPresent()) {
      var controlPos = headingControl.get().get();
      var globalPose = GlobalPosition.Get();
      var directionToTarget = controlPos.getTranslation().minus(globalPose.getTranslation());
      // Calculate desired heading to face the target
      var desiredHeading = Math.atan2(directionToTarget.getY(), directionToTarget.getX());
      var currentHeading = globalPose.getRotation().getRadians();

      // Controller outputs angular velocity in rad/s, convert to percentage [-1, 1]
      double headingOutputRadPerSec = headingController.calculate(currentHeading, desiredHeading);
      double headingOutputPercent = headingOutputRadPerSec / c.kMaxAngularSpeedRadPerSec;

      Logger.recordOutput("Swerve/HeadingControl/CurrentHeading", currentHeading);
      Logger.recordOutput("Swerve/HeadingControl/DesiredHeading", desiredHeading);
      Logger.recordOutput("Swerve/HeadingControl/OutputRadPerSec", headingOutputRadPerSec);
      Logger.recordOutput("Swerve/HeadingControl/OutputPercent", headingOutputPercent);

      r = headingOutputPercent;
    }

    var velocity = SwerveSubsystem.fromPercentToVelocity(new Vec2(x, y), r);
    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.GYRO_RELATIVE);
  }

  public void toggleHeadingControl() {
    isHeadingControl = !isHeadingControl;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }
}
