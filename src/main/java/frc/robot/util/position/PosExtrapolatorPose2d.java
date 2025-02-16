package frc.robot.util.position;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PosExtrapolatorPose2d extends Pose2d {
    public PosExtrapolatorPose2d(float x, float y, Rotation2d rotation2d) {
        super(x, y, rotation2d);
    }

    /**
     * @note it is assumed that the current position is in pos estimator orientation (forward = -y, right = +x)
     * @orientation forward = +x, right = -y
     * @return swerve odometry based position
     */
    public Pose2d getSwerveRelative() {
        return new Pose2d(-this.getY(), -this.getX(), new Rotation2d(
                this.getRotation().getSin(),
                this.getRotation().getCos()));
    }

    /**
     * @note it is assumed that the current position is in swerve relative
     * @orientation forward = -y, right = +x
     * @return pos extrapolator relative position
     */
    public Pose2d getPosExtrapolatorRelative() {
        return new Pose2d(-this.getX(), -this.getY(), new Rotation2d(
                this.getRotation().getSin(),
                this.getRotation().getCos()));
    }
}
