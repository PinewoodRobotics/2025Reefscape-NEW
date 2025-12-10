package frc.robot.command.auto_driving_commands.driving;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;

public class ExecuteStraightTrajectory extends Command {
  private final Pose2d target;
  private final HolonomicDriveController hdc;
  private final Timer timer = new Timer();
  private final TrapezoidProfile velocityProfile;
  private Translation2d startTrans;
  private double totalDistance;
  private TrapezoidProfile.State startState;
  private TrapezoidProfile.State endState;
  private double toleranceDistance;
  private double toleranceAngleDegrees;

  public ExecuteStraightTrajectory(Pose2d target, double maxSpeed, double maxAcceleration, double maxOmega,
      double maxAlpha, double toleranceDistance, double toleranceAngleDegrees) {
    super();
    this.target = target;
    this.velocityProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));

    this.hdc = new HolonomicDriveController(
        new PIDController(5, 0.0, 0.2), // kP, kI, kD for X position
        new PIDController(5, 0.0, 0.2), // kP, kI, kD for Y position
        new ProfiledPIDController(1.0, 0.0, 0,
            new TrapezoidProfile.Constraints(maxOmega, maxAlpha)));

    addRequirements(SwerveSubsystem.GetInstance());
  }

  public ExecuteStraightTrajectory(Translation2d target, double maxSpeed, double maxAcceleration, double maxOmega,
      double maxAlpha, double toleranceDistance, double toleranceAngleDegrees) {
    this(new Pose2d(target, GlobalPosition.Get().getRotation()), maxSpeed, maxAcceleration, maxOmega, maxAlpha,
        toleranceDistance,
        toleranceAngleDegrees);
  }

  @Override
  public void initialize() {
    Translation2d start = GlobalPosition.Get().getTranslation();
    Translation2d end = target.getTranslation();
    totalDistance = start.minus(end).getNorm();
    startState = new TrapezoidProfile.State(0.0, 0.0);
    endState = new TrapezoidProfile.State(totalDistance, 0.0);
    startTrans = start;

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    Pose2d current = GlobalPosition.Get();
    TrapezoidProfile.State nextState = velocityProfile.calculate(timer.get(), startState, endState);
    double distAlongLine = nextState.position;
    double velAlongLine = nextState.velocity;
    Translation2d direction = target.getTranslation().minus(startTrans).div(totalDistance);
    Translation2d position = startTrans.plus(direction.times(distAlongLine));

    Pose2d nextPose = new Pose2d(position, target.getRotation());
    var speeds = hdc.calculate(
        current,
        nextPose,
        velAlongLine,
        target.getRotation());

    SwerveSubsystem.GetInstance().drive(speeds, SwerveSubsystem.DriveType.RAW);
  }

  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.GetInstance().stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    boolean reachedGoal = Math
        .abs(GlobalPosition.Get().getTranslation().minus(target.getTranslation()).getNorm()) < toleranceDistance;
    boolean atRest = Math.abs(SwerveSubsystem.GetInstance().getChassisSpeeds().vxMetersPerSecond) < 0.05;
    boolean atAngle = Math.abs(
        GlobalPosition.Get().getRotation().getDegrees() - target.getRotation().getDegrees()) < toleranceAngleDegrees;

    return reachedGoal && atRest && atAngle;
  }
}
