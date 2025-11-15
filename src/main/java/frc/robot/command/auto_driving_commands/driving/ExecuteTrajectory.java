package frc.robot.command.auto_driving_commands.driving;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import proto.pathfind.Pathfind.PathfindResult;

public class ExecuteTrajectory extends Command {
  private static final double MAX_OMEGA = Math.PI / 1.8; // rad/s (theta constraints)
  private static final double MAX_ALPHA = 1; // rad/s^2
  private static final double MAX_VEL_MPS = 2.0;
  private static final double MAX_ACCEL_MPS2 = 0.9;
  private static final double EXTRA_TIME_SECONDS = 0.5; // Extra time after trajectory completes

  private final Timer timer = new Timer();
  private final Supplier<Trajectory> trajectorySupplier;
  private final Optional<Rotation2d> targetRotation;

  private Trajectory trajectory;

  private final HolonomicDriveController hdc = new HolonomicDriveController(
      new PIDController(5, 0.0, 0.2), // kP, kI, kD for X position
      new PIDController(5, 0.0, 0.2), // kP, kI, kD for Y position
      new ProfiledPIDController(1.0, 0.0, 0,
          new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA)));

  public ExecuteTrajectory(Supplier<Trajectory> trajectory, Optional<Rotation2d> targetRotation) {
    this.trajectorySupplier = trajectory;
    this.targetRotation = targetRotation;
    ((ProfiledPIDController) hdc.getThetaController()).enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(SwerveSubsystem.GetInstance());
  }

  public ExecuteTrajectory(Trajectory trajectory, Optional<Rotation2d> targetRotation) {
    this(() -> trajectory, targetRotation);
  }

  public ExecuteTrajectory(Pose2d target, boolean useFacingGoal, boolean useTrajectoryRotation) {
    this(() -> generateTrajectoryToTarget(GlobalPosition.Get(), target, useFacingGoal),
        useTrajectoryRotation ? Optional.of(target.getRotation()) : Optional.empty());
  }

  public ExecuteTrajectory(Pose2d target, boolean useFacingGoal) {
    this(target, useFacingGoal, true);
  }

  public ExecuteTrajectory(Pose2d target) {
    this(target, true, true);
  }

  public ExecuteTrajectory(PathfindResult pathfindResult) {
    this(() -> CustomMath.generatePathfindingTrajectory(
        CustomMath.fromPathfindResultToTranslation2dList(pathfindResult),
        MAX_VEL_MPS,
        MAX_ACCEL_MPS2), Optional.empty());
  }

  public ExecuteTrajectory(PathfindResult pathfindResult, Rotation2d rotation) {
    this(() -> CustomMath.generatePathfindingTrajectory(
        CustomMath.fromPathfindResultToTranslation2dList(pathfindResult),
        MAX_VEL_MPS,
        MAX_ACCEL_MPS2), Optional.of(rotation));
  }

  private static Trajectory generateTrajectoryToTarget(Pose2d start, Pose2d target, boolean useFacingGoal) {
    Rotation2d angleToGoal = CustomMath.getRotationToNextPoint(start.getTranslation(), target.getTranslation());
    Pose2d startFacingGoal = new Pose2d(start.getTranslation(), angleToGoal);

    return TrajectoryGenerator.generateTrajectory(
        useFacingGoal ? startFacingGoal : start,
        List.of(),
        target,
        new TrajectoryConfig(MAX_VEL_MPS, MAX_ACCEL_MPS2));
  }

  @Override
  public void initialize() {
    trajectory = trajectorySupplier.get();

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double currentTime = timer.get();

    Pose2d current = GlobalPosition.Get();

    Trajectory.State state = trajectory.sample(currentTime);

    ChassisSpeeds fieldRelativeSpeeds = hdc.calculate(
        current,
        state.poseMeters,
        state.velocityMetersPerSecond,
        targetRotation.orElse(current.getRotation()));

    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        current.getRotation());

    SwerveSubsystem.GetInstance().drive(robotRelativeSpeeds, SwerveSubsystem.DriveType.RAW);

    Logger.recordOutput("ExecuteTrajectory/desiredPose", state.poseMeters);
    Logger.recordOutput("ExecuteTrajectory/currentPose", current);
    Logger.recordOutput("ExecuteTrajectory/fieldRelativeSpeeds", new double[] {
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond });
    Logger.recordOutput("ExecuteTrajectory/robotRelativeSpeeds", new double[] {
        robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond });
    Logger.recordOutput("ExecuteTrajectory/trajectoryTime", trajectory);
  }

  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.GetInstance().stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    // Finish when trajectory time is complete plus a small buffer for settling
    return timer.get() > trajectory.getTotalTimeSeconds() + EXTRA_TIME_SECONDS;
  }
}
