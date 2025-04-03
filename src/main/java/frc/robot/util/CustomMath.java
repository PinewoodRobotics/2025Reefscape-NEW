package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.pwrup.util.Vec2;

/**
 * @note MathFun = Math Functions
 * @apiNote this is the file where all of the math functions go
 */
public class CustomMath {

  /**
   * @param x     the X position on the graph
   * @param yCoef the the height of the graph on the yAxis. This will increase /
   *              decrease the return val
   * @return The Y of the X on the graph
   */
  public static double getPosOnGraph(double x, double yCoef) {
    return Math.cbrt(x) + yCoef;
  }

  /**
   * @param values double values for which you want to get the biggest value
   * @return the max value form the input values
   */
  public static double max(double... values) {
    double m = values[0];

    for (double value : values) {
      if (value > m) {
        m = value;
      }
    }

    return m;
  }

  /**
   * @param values
   * @return the minimum value from the values
   */
  public static double min(double... values) {
    double m = values[0];
    for (double value : values) {
      if (value < m) {
        m = value;
      }
    }

    return m;
  }

  /**
   * Deadbands joystick input, then scales it from the deadband to 1. Ask Jared
   * for clarification.
   *
   * @param input    the joystick input, [0, 1]
   * @param deadband ignores the input if it is less than this value, [0, 1]
   * @param minValue adds this value if the input overcomes the deadband, [0, 1]
   * @return the return value, [0, 1]
   */
  public static double deadband(
    double input,
    double deadband,
    double minValue
  ) {
    double output;
    double m = (1.0 - minValue) / (1.0 - deadband);

    if (Math.abs(input) < deadband) {
      output = 0;
    } else if (input > 0) {
      output = m * (input - deadband) + minValue;
    } else {
      output = m * (input + deadband) - minValue;
    }

    return output;
  }

  public static double putWithinHalfToHalf(double in) {
    while (in > 0.5) {
      in -= 1;
    }
    while (in < -0.5) {
      in += 1;
    }
    return in;
    // return ((in + 0.5) % 1) - 0.5;
  }

  public static double wrapTo180(double angle) {
    double newAngle = (angle + 180) % 360;
    while (newAngle < 0) {
      newAngle += 360;
    }

    return newAngle - 180;
  }

  /**
   * @param angle1 the first angle
   * @param angle2 the second angle
   * @return the difference between the two angles within -180 to 180
   */
  public static double angleDifference180(double angle1, double angle2) {
    angle1 = ((angle1 + 180) % 360) - 180;
    angle2 = ((angle2 + 180) % 360) - 180;

    double diff = Math.abs(angle1 - angle2);
    return Math.min(diff, 360 - diff);
  }

  /**
   * Wraps a number within a custom range using a sigmoid-like curve for smoother transitions.
   *
   * @param currentNumber The input number to be wrapped
   * @param maxNumber The maximum value of the range
   * @param minNumber The minimum value of the range
   * @param wrapNumberPlusMinus The size of one complete wrap cycle
   * @return A number wrapped within the specified range [minNumber, maxNumber] with smooth transitions
   */
  public static double wrapSigmoid(
    double currentNumber,
    double maxNumber,
    double minNumber,
    double wrapNumberPlusMinus
  ) {
    double diff = currentNumber - minNumber;
    double wrap = (diff / wrapNumberPlusMinus) % 1;

    // Apply sigmoid-like smoothing using sine function
    wrap = (1 - Math.cos(wrap * Math.PI)) / 2;

    return wrap * (maxNumber - minNumber) + minNumber;
  }

  public class EasingFunctions {

    public static double easeOutCubic(
      double maxValue,
      double minValue,
      double currentValue,
      double maxY,
      double minY
    ) {
      double t;
      if (maxValue == minValue) {
        t = 0.0;
      } else {
        t = (currentValue - minValue) / (maxValue - minValue);
      }
      t = clamp(t, 0.0, 1.0);

      double easedValue = 1 - Math.pow(1 - t, 3);
      return minY + easedValue * (maxY - minY);
    }

    /**
     * Applies an ease-out quintic interpolation.
     *
     * @param input       The current value (e.g. time or progress)
     * @param inputStart  The start of the input range
     * @param inputEnd    The end of the input range
     * @param outputStart The start of the output range (e.g. min value)
     * @param outputEnd   The end of the output range (e.g. max value)
     * @param steepness   Steepness control (1 = normal quint, >1 = steeper, <1 = gentler)
     * @return The eased output value
     */
    public static double easeOutQuint(
      double input,
      double inputStart,
      double inputEnd,
      double outputStart,
      double outputEnd,
      double steepness
    ) {
      if (inputStart == inputEnd) {
        throw new IllegalArgumentException(
          "inputStart and inputEnd cannot be the same"
        );
      }

      double t = (input - inputStart) / (inputEnd - inputStart);
      t = Math.max(0, Math.min(1, t));

      double eased = 1 - Math.pow(1 - t, 5 * steepness);

      return outputStart + (outputEnd - outputStart) * eased;
    }

    private static double clamp(double val, double min, double max) {
      return Math.max(min, Math.min(max, val));
    }
  }

  public static Translation2d scaleToLength(
    Translation2d vector,
    double targetLength
  ) {
    if (vector.getNorm() == 0) {
      return new Translation2d(0, 0); // Avoid divide-by-zero
    }

    return vector.div(vector.getNorm()).times(targetLength);
  }

  public static double invertRadians(double initial) {
    return initial > 0 ? initial - Math.PI : initial + Math.PI;
  }

  public static SimpleMatrix fromPose2dToMatrix(Pose2d pose) {
    return new SimpleMatrix(
      new double[][] {
        {
          pose.getRotation().getCos(),
          -pose.getRotation().getSin(),
          pose.getX(),
        },
        {
          pose.getRotation().getSin(),
          pose.getRotation().getCos(),
          pose.getY(),
        },
        { 0, 0, 1 },
      }
    );
  }

  public static SimpleMatrix createTransformationMatrix(
    SimpleMatrix rotation,
    SimpleMatrix translation
  ) {
    SimpleMatrix result = new SimpleMatrix(4, 4);

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.set(i, j, rotation.get(i, j));
      }
    }

    for (int i = 0; i < 3; i++) {
      result.set(i, 3, translation.get(i, 0));
    }

    result.set(3, 3, 1.0);

    return result;
  }

  public static SimpleMatrix from3dTransformationMatrixTo2d(
    SimpleMatrix matrix
  ) {
    // matrix.print();

    return new SimpleMatrix(
      new double[][] {
        { matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 3) },
        { matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 3) },
        { 0, 0, 1 },
      }
    );
  }

  public static SimpleMatrix fromFloatList(
    List<Float> flatList,
    int rows,
    int cols
  ) {
    if (flatList == null || flatList.size() != rows * cols) {
      throw new IllegalArgumentException(
        "The provided list does not match the specified dimensions."
      );
    }

    var matrix = new SimpleMatrix(rows, cols);
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        matrix.set(i, j, flatList.get(i * cols + j));
      }
    }

    return matrix;
  }

  public static Pose2d fromTransformationMatrix3dToPose2d(SimpleMatrix matrix) {
    return new Pose2d(
      matrix.get(0, 3),
      matrix.get(1, 3),
      new Rotation2d(matrix.get(0, 0), matrix.get(1, 0))
    );
  }

  public static Pose2d fromTransformationMatrix2dToPose2d(SimpleMatrix matrix) {
    return new Pose2d(
      matrix.get(0, 2),
      matrix.get(1, 2),
      new Rotation2d(matrix.get(0, 0), matrix.get(1, 0))
    );
  }

  public static double plusMinusHalf(double in) {
    while (in > 0.5) {
      in -= 1;
    }
    while (in < -0.5) {
      in += 1;
    }
    return in;
  }

  public static double plusMinus180(double in) {
    while (in > 180) {
      in -= 360;
    }
    while (in < -180) {
      in += 360;
    }
    return in;
  }

  /**
   * For setpoint ramping, limits the change in setpoint by the maxRamp
   *
   * @param setpoint Where you wants your setpoint to be
   * @param currentSetpoint Where your setpoint currently is
   * @param maxRamp How fast you want your setpoint to be able to change, in units / tick
   * @return The new current setpoint
   */
  public static double rampSetpoint(
    double setpoint,
    double currentSetpoint,
    double maxRamp
  ) {
    if (setpoint - currentSetpoint > maxRamp) {
      return currentSetpoint += maxRamp;
    } else if (setpoint - currentSetpoint < -maxRamp) {
      return currentSetpoint -= maxRamp;
    }
    return currentSetpoint = setpoint;
  }

  /**
   * @param tagPose the position of the tag in the ROBOT's view. It is ok if that changes.
   * @param alignment the TAG RELATIVE position to where you want to go (for example [-1, 0] will get you 1 meter in front of the tag directly in the center.)
   * @return the direction vector where the wheels have to move to to get to the alignment pose
   */
  public static Pose2d finalPointDirection(Pose2d tagPose, Pose2d alignment) {
    return new Pose2d(tagPose.toMatrix().times(alignment.toMatrix()));
  }

  /**
   *
   * @param diffRadians the difference between two angles in radians
   * @param rangeRadians the error thingy where you want to stop (like if you put 20 deg here, you will be in +-20 deg of target)
   * @return -1 = left rotation, 1 = right rotation, 0 = stop rotating alltogether
   */
  public static int rotationDirection(double diffRadians, double rangeRadians) {
    if (diffRadians > rangeRadians) {
      return -1;
    } else if (diffRadians < -rangeRadians) {
      return 1;
    } else {
      return 0;
    }
  }

  public static class DrivingMath {

    public static Vec2 calculateDirectionVector(Pose2d finalPose) {
      return new Vec2((float) finalPose.getX(), (float) -finalPose.getY())
        .scaleToModulo(1);
    }

    public static int calculateRotationDirection(
      Pose2d finalPose,
      DriveConfig driveConfig
    ) {
      double diff = finalPose.getRotation().getRadians();
      if (diff > Math.toRadians(driveConfig.getAngularStoppingDistanceDeg())) {
        return -1;
      } else if (
        diff < -Math.toRadians(driveConfig.getAngularStoppingDistanceDeg())
      ) {
        return 1;
      }
      return 0;
    }

    public static double calculateSpeed(
      double distance,
      DriveConfig driveConfig,
      SlowdownConfig slowdownConfig
    ) {
      double speed =
        driveConfig.getMaxTranslationSpeed() *
        slowdownConfig.getFirstTierMaxSpeedMultiplier();
      if (distance < slowdownConfig.getSecondTierDistance()) {
        speed =
          driveConfig.getMaxTranslationSpeed() *
          slowdownConfig.getSecondTierMaxSpeedMultiplier();
      }
      if (distance < slowdownConfig.getThirdTierDistance()) {
        speed =
          driveConfig.getMaxTranslationSpeed() *
          slowdownConfig.getThirdTierMaxSpeedMultiplier();
      }

      return speed;
    }
  }
}
