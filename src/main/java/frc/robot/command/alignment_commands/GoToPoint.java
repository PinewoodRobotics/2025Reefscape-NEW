package frc.robot.command.alignment_commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;
import lombok.Getter;
import lombok.Setter;

public class GoToPoint extends Command {
  private final double maxRotationSpeed = 0.6;
  private final double maxDriveSpeed = 0.8;
  private final double minRotationSpeed = 0.05;
  private final double minDriveSpeed = 0.06;

  private final double rotationThresholdDeg = 2.0;
  private final double distanceThresholdM = 0.05;

  private final double maxRotationRangeDeg = 45.0;
  private final double maxDistanceRangeM = 5.0;

  private final SwerveSubsystem swerve = SwerveSubsystem.GetInstance();

  private final Supplier<Translation2d> targetTranslationSupplier;
  private final Supplier<Rotation2d> targetHeadingSupplier;

  private final SlewRateLimiter driveSlewLimiter = new SlewRateLimiter(2.0);

  @Setter
  @Getter
  @AutoLog
  public static class GoToPointState {
    public Translation2d targetFieldPoint = new Translation2d();
    public Rotation2d targetHeading = new Rotation2d();
    public Pose2d currentPose = new Pose2d();

    public boolean hasPose = false;
    public boolean aligning = false;

    public double distanceRemainingM = 0.0;
    public double rotationRemainingDeg = 0.0;

    public double calculatedDriveSpeed = 0.0;
    public double calculatedRotationSpeed = 0.0;
    public int rotationDirection = 0;
  }

  private final GoToPointStateAutoLogged state = new GoToPointStateAutoLogged();

  public GoToPoint(Translation2d fieldPoint) {
    this(() -> fieldPoint, null);
  }

  public GoToPoint(Translation2d fieldPoint, Rotation2d finalHeading) {
    this(() -> fieldPoint, () -> finalHeading);
  }

  public GoToPoint(Supplier<Translation2d> fieldPointSupplier, Supplier<Rotation2d> finalHeadingSupplier) {
    this.targetTranslationSupplier = fieldPointSupplier;
    this.targetHeadingSupplier = finalHeadingSupplier;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    state.setAligning(true);
    driveSlewLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    Pose2d pose = GlobalPosition.Get();
    state.setHasPose(pose != null);
    if (pose == null) {
      swerve.driveRaw(new Vec2(1, 0), 0, 0);
      state.setAligning(false);
      Logger.processInputs("GoToPoint", state);
      return;
    }

    state.setCurrentPose(pose);

    Translation2d targetPoint = targetTranslationSupplier.get();
    state.setTargetFieldPoint(targetPoint);
    Rotation2d holdOrGoalHeading = (targetHeadingSupplier != null)
        ? targetHeadingSupplier.get()
        : pose.getRotation();
    state.setTargetHeading(holdOrGoalHeading);

    double dx = targetPoint.getX() - pose.getX();
    double dy = targetPoint.getY() - pose.getY();
    Translation2d error = new Translation2d(dx, dy);

    double distance = error.getNorm();
    double driveSpeed = calculateProportionalDriveSpeed(distance);
    double filteredDriveSpeed = driveSlewLimiter.calculate(driveSpeed);

    double rotationErrorDeg = holdOrGoalHeading.minus(pose.getRotation()).getDegrees();
    double rotationSpeed = calculateProportionalRotationSpeed(Math.abs(rotationErrorDeg));
    int rotationDirection = getOptimalDirectionRotate(pose.getRotation(), holdOrGoalHeading, rotationThresholdDeg);

    state.setDistanceRemainingM(distance);
    state.setRotationRemainingDeg(Math.abs(rotationErrorDeg));
    state.setCalculatedDriveSpeed(filteredDriveSpeed);
    state.setCalculatedRotationSpeed(rotationSpeed);
    state.setRotationDirection(rotationDirection);

    swerve.drive(
        SwerveSubsystem.toSwerveOrientation(error),
        rotationDirection * rotationSpeed,
        filteredDriveSpeed,
        GlobalPosition.Get().getRotation().getRadians());

    boolean atPos = distance <= distanceThresholdM;
    boolean atAng = Math.abs(rotationErrorDeg) <= rotationThresholdDeg;
    state.setAligning(!(atPos && atAng));

    Logger.processInputs("GoToPoint", state);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveRaw(new Vec2(1, 0), 0, 0);
    state.setAligning(false);
  }

  @Override
  public boolean isFinished() {
    return !state.isAligning();
  }

  private int getOptimalDirectionRotate(Rotation2d current, Rotation2d target, double thresholdDegrees) {
    double angleDifference = target.minus(current).getRadians();
    double absDeg = Math.abs(Math.toDegrees(angleDifference));
    state.setRotationRemainingDeg(absDeg);

    if (absDeg < thresholdDegrees)
      return 0;
    return (angleDifference > 0) ? -1 : 1;
  }

  private double calculateProportionalDriveSpeed(double distance) {
    if (distance <= distanceThresholdM)
      return 0.0;

    double normalized = Math.min(distance / maxDistanceRangeM, 1.0);
    double speed = minDriveSpeed + (maxDriveSpeed - minDriveSpeed) * normalized;
    return clamp(speed, minDriveSpeed, maxDriveSpeed);
  }

  private double calculateProportionalRotationSpeed(double rotationErrorDeg) {
    if (rotationErrorDeg <= rotationThresholdDeg)
      return 0.0;

    double normalized = Math.min(rotationErrorDeg / maxRotationRangeDeg, 1.0);
    double speed = minRotationSpeed + (maxRotationSpeed - minRotationSpeed) * normalized;
    return clamp(speed, minRotationSpeed, maxRotationSpeed);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}