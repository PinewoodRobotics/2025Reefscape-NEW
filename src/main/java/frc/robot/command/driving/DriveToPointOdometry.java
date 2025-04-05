package frc.robot.command.driving;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import frc.robot.util.CustomMath.DrivingMath;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;

public class DriveToPointOdometry extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final Pose2d m_targetPose;
  private final OdometrySubsystem m_odometrySubsystem;
  private final DriveConfig driveConfig;
  private final SlowdownConfig slowdownConfig;
  private boolean isEnabled;

  public DriveToPointOdometry(
      SwerveSubsystem m_swerveSubsystem,
      Pose2d m_targetPose,
      DriveConfig driveConfig,
      SlowdownConfig slowdownConfig,
      OdometrySubsystem m_odometrySubsystem,
      boolean isEnabled) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_targetPose = m_targetPose;
    this.isEnabled = isEnabled;
    this.m_odometrySubsystem = m_odometrySubsystem;
    this.driveConfig = driveConfig;
    this.slowdownConfig = slowdownConfig;

    addRequirements(m_swerveSubsystem);
  }

  public void setEnabled(boolean isEnabled) {
    this.isEnabled = isEnabled;
  }

  @Override
  public void initialize() {
    isEnabled = true;
  }

  @Override
  public void execute() {
    if (!isEnabled) {
      return;
    }

    var matrix = CustomMath.fromPose2dToMatrix(m_odometrySubsystem.latestPosition).invert()
        .mult(CustomMath.fromPose2dToMatrix(m_targetPose));

    matrix.print();

    var position = CustomMath.fromTransformationMatrix2dToPose2d(matrix);

    var direction = new Vec2(-position.getX(), position.getY())
        .scaleToModulo(1);
    var rotationDirection = DrivingMath.calculateRotationDirection(
        position,
        driveConfig);
    var distance = position.getTranslation().getNorm();
    var speed = DrivingMath.calculateSpeed(
        distance,
        driveConfig,
        slowdownConfig);

    m_swerveSubsystem.driveRaw(direction, rotationDirection, speed);
    if (distance < driveConfig.getTranslationStoppingDistance() &&
        rotationDirection == 0) {
      end(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.isEnabled = false;
    m_swerveSubsystem.driveRaw(new Vec2(0, 0), 0, 0);
  }
}
