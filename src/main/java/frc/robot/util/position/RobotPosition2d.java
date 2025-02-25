package frc.robot.util.position;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotPosition2d extends Pose2d {
  public final RobotPositionType positionType;

  public RobotPosition2d(double x, double y, Rotation2d rotation2d, RobotPositionType positionType) {
    super(x, y, rotation2d);
    this.positionType = positionType;
  }

  public RobotPosition2d(Pose2d pos, RobotPositionType positionType) {
    this(pos.getX(), pos.getY(), pos.getRotation(), positionType);
  }

  public RobotPosition2d() {
    this(0.0, 0.0, new Rotation2d(), RobotPositionType.GLOBAL);
  }

  /**
   * @note it is assumed that the current position is in pos estimator orientation (forward = -y, right = +x)
   * @orientation forward = +x, right = -y
   * @return swerve odometry based position
   */
  public RobotPosition2d getSwerveRelative() {
    if (positionType == RobotPositionType.SWERVE) {
      return this;
    }

    return new RobotPosition2d(-this.getY(), -this.getX(), new Rotation2d(
        this.getRotation().getCos(),
        this.getRotation().getSin()), RobotPositionType.SWERVE);
  }

  /**
   * @note it is assumed that the current position is in swerve relative
   * @orientation forward = -y, right = +x
   * @return pos extrapolator relative position
   */
  public RobotPosition2d getGlobalPosition() {
    if (positionType == RobotPositionType.GLOBAL) {
      return this;
    }

    return new RobotPosition2d(-this.getX(), -this.getY(), new Rotation2d(
        this.getRotation().getCos(),
        this.getRotation().getSin()), RobotPositionType.GLOBAL);
  }
}
