package frc.robot.util.position;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;

public class RobotPosition2d extends Pose2d {
  public final Orientation positionOrientation;

  public RobotPosition2d(double x, double y, Rotation2d rotation2d, Orientation positionOrientation) {
    super(x, y, rotation2d);
    this.positionOrientation = positionOrientation;
  }

  public RobotPosition2d(Pose2d pos, Orientation positionOrientation) {
    this(pos.getX(), pos.getY(), pos.getRotation(), positionOrientation);
  }

  public RobotPosition2d(Transform2d transform2d, Orientation positionOrientation) {
    this(transform2d.getX(), transform2d.getY(), transform2d.getRotation(), positionOrientation);
  }

  public RobotPosition2d() {
    this(0.0, 0.0, new Rotation2d(), Orientation.GLOBAL);
  }

  public Pose2d toPose2d() {
    return this;
  }

  public Matrix<N3, N3> getTransformationMatrix() {
    double cosTheta = this.getRotation().getCos();
    double sinTheta = this.getRotation().getSin();
    double x = this.getX();
    double y = this.getY();

    return new Matrix<>(Nat.N3(), Nat.N3(), new double[] {
        cosTheta, -sinTheta, x,
        sinTheta, cosTheta, y,
        0, 0, 1
    });
  }

  public RobotPosition2d fromFieldToSwerve() {
    assert this.positionOrientation == Orientation.FIELD;
    return new RobotPosition2d(this.getY(), -this.getX(), this.getRotation(), Orientation.SWERVE);
  }

  public RobotPosition2d fromSwerveToField() {
    assert this.positionOrientation == Orientation.SWERVE;
    return new RobotPosition2d(-this.getY(), this.getX(), this.getRotation(), Orientation.FIELD);
  }

  public RobotPosition2d fromPosExtrapolatorToField() {
    assert this.positionOrientation == Orientation.GLOBAL;
    return new RobotPosition2d(this.getX(), -this.getY(), this.getRotation(), Orientation.FIELD);
  }

  public RobotPosition2d fromFieldToPosExtrapolator() {
    assert this.positionOrientation == Orientation.FIELD;
    return new RobotPosition2d(this.getX(), -this.getY(), this.getRotation(), Orientation.GLOBAL);
  }

  public RobotPosition2d switchSinCos() {
    return new RobotPosition2d(
        new Pose2d(this.getTranslation(), new Rotation2d(this.getRotation().getSin(), this.getRotation().getCos())),
        positionOrientation);
  }
}
