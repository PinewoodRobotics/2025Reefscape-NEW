package frc.robot.command.alignment_commands;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToSetpointExact extends Command {
  private final Pose2d target;
  private final Timer timer = new Timer();

  private static final double MAX_VEL_MPS = 2.0; // cap below mech max for smoothness
  private static final double MAX_ACCEL_MPS2 = 0.5;
  private static final double MAX_OMEGA = Math.PI / 1.8; // rad/s (theta constraints)
  private static final double MAX_ALPHA = 1; // rad/s^2

  private final HolonomicDriveController hdc = new HolonomicDriveController(
      new PIDController(5, 0.0, 0.2), // kP, kI, kD for X position
      new PIDController(5, 0.0, 0.2), // kP, kI, kD for Y position
      new ProfiledPIDController(1.0, 0.0, 0,
          new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA)));

  private Trajectory trajectory;

  public GoToSetpointExact(Pose2d target) {
    this.target = target;
    addRequirements(SwerveSubsystem.GetInstance());
    ((ProfiledPIDController) hdc.getThetaController()).enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d start = GlobalPosition.Get();

    // Calculate the angle towards the goal
    Translation2d toTarget = target.getTranslation().minus(start.getTranslation());
    Rotation2d angleToGoal = new Rotation2d(toTarget.getX(), toTarget.getY());

    // Create start pose facing the goal
    Pose2d startFacingGoal = new Pose2d(start.getTranslation(), angleToGoal);

    trajectory = TrajectoryGenerator.generateTrajectory(
        startFacingGoal,
        List.of(),
        target,
        new TrajectoryConfig(MAX_VEL_MPS, MAX_ACCEL_MPS2));

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
        target.getRotation());

    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        current.getRotation());

    Logger.recordOutput("GoToSetpoint/desiredPose", state.poseMeters);
    Logger.recordOutput("GoToSetpoint/currentPose", current);
    Logger.recordOutput("GoToSetpoint/targetPose", target);
    Logger.recordOutput("GoToSetpoint/fieldRelativeSpeeds", new double[] {
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond });
    Logger.recordOutput("GoToSetpoint/robotRelativeSpeeds", new double[] {
        robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond });
    Logger.recordOutput("GoToSetpoint/trajectoryTime", trajectory);

    SwerveSubsystem.GetInstance().drive(robotRelativeSpeeds, SwerveSubsystem.DriveType.RAW);
  }

  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.GetInstance().stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if we've reached the goal distance and HDC is at reference
    boolean reachedGoal = Math
        .abs(GlobalPosition.Get().getTranslation().minus(target.getTranslation()).getNorm()) < 0.05; // 5cm
    // tolerance
    boolean atRest = Math.abs(SwerveSubsystem.GetInstance().getChassisSpeeds().vxMetersPerSecond) < 0.05; // 5cm/s
                                                                                                          // tolerance
    return reachedGoal && atRest && hdc.atReference();
  }
}
