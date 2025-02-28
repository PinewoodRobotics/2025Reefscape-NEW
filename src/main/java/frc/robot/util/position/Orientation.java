package frc.robot.util.position;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Enum representing the three coordinate system orientations.
 *
 * For each system we define a 3x3 homogeneous transformation matrix that
 * converts a vector from that coordinate system into the common physical
 * coordinate system (physical: +x = right, +y = forward). The inverse of that
 * matrix (provided by getFromPhysicalMatrix()) converts back.
 * 
 * @note PLEASE NOTE THAT I DO NOT IN ANY WAY ENDORSE THIS TYPE OF CONVERSIONS AND STUFF!! THIS IS JUST BECAUSE OF BAD COORDINATION
 * AND OTHER THINGS WHICH LED TO THIS BAD STRUCTURE :<.
 */
public enum Orientation {
  FIELD {
    // FIELD: +x = right, +y = forward.
    @Override
    public Matrix<N3, N3> getToPhysicalMatrix() {
      return MatBuilder.fill(N3.instance, N3.instance, 1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    @Override
    public Matrix<N3, N3> getFromPhysicalMatrix() {
      // Identity is self-inverse.
      return MatBuilder.fill(N3.instance, N3.instance, 1, 0, 0, 0, 1, 0, 0, 0, 1);
    }
  },
  SWERVE {
    // SWERVE: +x = forward, +y = left.
    // A point (x, y) in SWERVE maps to physical (-y, x), a 90° CCW rotation.
    @Override
    public Matrix<N3, N3> getToPhysicalMatrix() {
      return MatBuilder.fill(N3.instance, N3.instance, 0, -1, 0, 1, 0, 0, 0, 0, 1);
    }

    @Override
    public Matrix<N3, N3> getFromPhysicalMatrix() {
      // Inverse is a 90° clockwise rotation.
      return MatBuilder.fill(N3.instance, N3.instance, 0, 1, 0, -1, 0, 0, 0, 0, 1);
    }
  },
  GLOBAL {
    // GLOBAL: +x = left, +y = back.
    // A point (x, y) in GLOBAL maps to physical (-x, -y), a 180° rotation.
    @Override
    public Matrix<N3, N3> getToPhysicalMatrix() {
      return MatBuilder.fill(N3.instance, N3.instance, -1, 0, 0, 0, -1, 0, 0, 0, 1);
    }

    @Override
    public Matrix<N3, N3> getFromPhysicalMatrix() {
      // 180° rotation is its own inverse.
      return MatBuilder.fill(N3.instance, N3.instance, -1, 0, 0, 0, -1, 0, 0, 0, 1);
    }
  };

  public abstract Matrix<N3, N3> getToPhysicalMatrix();

  public abstract Matrix<N3, N3> getFromPhysicalMatrix();

  /**
   * Converts a Transform2d (translation + rotation) into a 3x3 homogeneous matrix.
   * The matrix is:
   *
   *   [ cosθ  -sinθ   tx ]
   *   [ sinθ   cosθ   ty ]
   *   [  0      0     1  ]
   *
   * @param transform the Transform2d to convert.
   * @return the corresponding 3x3 matrix.
   */
  public static Matrix<N3, N3> transform2dToMatrix(Transform2d transform) {
    double cos = transform.getRotation().getCos();
    double sin = transform.getRotation().getSin();
    double tx = transform.getTranslation().getX();
    double ty = transform.getTranslation().getY();
    return MatBuilder
        .fill(N3.instance, N3.instance, cos, -sin, tx,
            sin, cos, ty,
            0, 0, 1);
  }

  /**
   * Converts a 3x3 homogeneous matrix back into a Transform2d.
   * Translation is extracted from the third column and rotation via atan2 from the matrix.
   *
   * @param matrix the 3x3 matrix.
   * @return the corresponding Transform2d.
   */
  public static Transform2d matrixToTransform2d(Matrix<N3, N3> matrix) {
    double tx = matrix.get(0, 2);
    double ty = matrix.get(1, 2);
    double cos = matrix.get(0, 0);
    double sin = matrix.get(1, 0);
    double angle = Math.atan2(sin, cos);
    return new Transform2d(new Translation2d(tx, ty), new Rotation2d(angle));
  }

  public static Transform2d convert(Transform2d transform, Orientation source, Orientation target) {
    return convert(transform2dToMatrix(transform), source, target);
  }

  /**
  * Converts a Transform2d from a source orientation to a target orientation using matrix multiplication.
  *
  * The conversion is:
  *
  *   T_target = (target.getFromPhysicalMatrix() * source.getToPhysicalMatrix()) * T_source
  *
  * @param transform the transform expressed in the source coordinate system.
  * @param source the source orientation.
  * @param target the target orientation.
  * @return the transform expressed in the target coordinate system.
  */
  public static Transform2d convert(Matrix<N3, N3> transform, Orientation source, Orientation target) {
    Matrix<N3, N3> M_source = source.getToPhysicalMatrix();
    Matrix<N3, N3> M_targetInv = target.getFromPhysicalMatrix();

    // Convert to the common physical coordinate system.
    Matrix<N3, N3> physicalMatrix = M_source.times(transform);
    // Convert from physical to the target system.
    Matrix<N3, N3> targetMatrix = M_targetInv.times(physicalMatrix);

    return matrixToTransform2d(targetMatrix);
  }
}