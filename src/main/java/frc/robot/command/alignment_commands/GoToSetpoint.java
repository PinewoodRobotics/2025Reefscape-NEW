package frc.robot.command.alignment_commands;

import java.util.List;
import java.util.function.Supplier;

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

public class GoToSetpoint extends Command {
  private final Supplier<Pose2d> target;
  private final Supplier<Pose2d> currentPosition;
  private final Timer timer = new Timer();

  private Trajectory traj;

  private static final double MAX_VEL_MPS = 1.0; // cap below mech max for smoothness
  private static final double MAX_ACCEL_MPS2 = 0.5;
  private static final double MAX_OMEGA = Math.PI * 2.0; // rad/s (theta constraints)
  private static final double MAX_ALPHA = Math.PI * 4.0; // rad/s^2

  private final HolonomicDriveController hdc = new HolonomicDriveController(
      new PIDController(2.0, 0.0, 0.0), // kP_X, add kD if you see oscillation
      new PIDController(2.0, 0.0, 0.0), // kP_Y
      new ProfiledPIDController(6.0, 0.0, 0.2,
          new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA)));

  public GoToSetpoint(Supplier<Pose2d> target, Supplier<Pose2d> currentPosition) {
    this.target = target;
    this.currentPosition = currentPosition;
    addRequirements(SwerveSubsystem.GetInstance());
    ((ProfiledPIDController) hdc.getThetaController()).enableContinuousInput(-Math.PI, Math.PI);
  }

  public GoToSetpoint(Pose2d target) {
    this(() -> target, () -> GlobalPosition.Get());
  }

  @Override
  public void initialize() {
    Pose2d start = GlobalPosition.Get();
    Pose2d goal = target.get();

    TrajectoryConfig cfg = new TrajectoryConfig(MAX_VEL_MPS, MAX_ACCEL_MPS2);
    cfg.setKinematics(SwerveSubsystem.GetInstance().getKinematics());

    traj = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(start.getTranslation(), start.getRotation()),
            new Pose2d(goal.getTranslation(), goal.getRotation())),
        cfg);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double t = Math.min(timer.get(), traj.getTotalTimeSeconds());
    Trajectory.State ref = traj.sample(t);
    Pose2d current = GlobalPosition.Get();

    Rotation2d desiredHeading = ref.poseMeters.getRotation();

    ChassisSpeeds speeds = hdc.calculate(
        current,
        ref.poseMeters, // desired pose at time t
        ref.velocityMetersPerSecond, // desired linear speed magnitude
        desiredHeading // desired robot heading
    );

    speeds.vxMetersPerSecond = clamp(speeds.vxMetersPerSecond, -MAX_VEL_MPS, MAX_VEL_MPS);
    speeds.vyMetersPerSecond = clamp(speeds.vyMetersPerSecond, -MAX_VEL_MPS, MAX_VEL_MPS);
    speeds.omegaRadiansPerSecond = clamp(speeds.omegaRadiansPerSecond, -MAX_OMEGA, MAX_OMEGA);

    // SwerveSubsystem.GetInstance().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.GetInstance().stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= traj.getTotalTimeSeconds()
        && hdc.atReference(); // small pose/angle tolerances internally
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
