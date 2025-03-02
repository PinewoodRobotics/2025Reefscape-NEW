package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TestOrientation {
  private static final double EPSILON = 1e-6;

  /**
   * Converts a world-space point into the robot's local coordinate system.
   * The robot's local frame is defined as:
   * @x-axis +x = forward. Essentially the direction vector defines x-axis
   * @y-axis +y = left essentially the direction vector -90deg clockwise.
   *
   * @param robotPose  The robot's Pose2d in world coordinates.
   * @param targetPose The target point's Pose2d in world coordinates.
   * @return The target's position in the robot's local frame as a Translation2d.
   */
  public static Translation2d worldToRobotFrame(Pose2d robotPose, Pose2d targetPose) {
    var robotRotation = new SimpleMatrix(new double[][] {
        { robotPose.getRotation().getCos(), robotPose.getRotation().getSin() },
        { -robotPose.getRotation().getSin(), robotPose.getRotation().getCos() }
    });

    var robotPositionGlobalFrame = new SimpleMatrix(new double[][] {
        { robotPose.getX() },
        { robotPose.getY() }
    });

    var targetPoseGlobalFrame = new SimpleMatrix(new double[][] {
        { targetPose.getX() },
        { targetPose.getY() }
    });

    SimpleMatrix displacementGlobal = targetPoseGlobalFrame.minus(robotPositionGlobalFrame);
    SimpleMatrix targetPoseLocalFrame = robotRotation.mult(displacementGlobal);

    System.out.println(robotRotation);
    System.out.println();
    System.out.println(targetPoseLocalFrame);

    return new Translation2d((float) targetPoseLocalFrame.get(0, 0), (float) targetPoseLocalFrame.get(1, 0));
  }

  @Test
  public void test180Degrees() {
    // Robot at (0,0) facing 180° (i.e. forward = (-1,0), right = (0,1))
    Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(-1, 0));
    // Target at (1,1)
    Pose2d targetPose = new Pose2d(2, 3, new Rotation2d());
    Translation2d result = worldToRobotFrame(robotPose, targetPose);
    assertEquals(-2.0, result.getX(), EPSILON, "X coordinate failed for 180°");
    assertEquals(-3.0, result.getY(), EPSILON, "Y coordinate failed for 180°");
  }

  @Test
  public void test0Degrees() {
    // Robot at (0,0) facing 0° (forward = (1,0), right = (0,-1))
    Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    // Target at (3,2)
    Pose2d targetPose = new Pose2d(3, 2, new Rotation2d());
    Translation2d result = worldToRobotFrame(robotPose, targetPose);
    System.out.println(result);

    assertEquals(3.0, result.getX(), EPSILON, "X coordinate failed for 0°");
    assertEquals(2.0, result.getY(), EPSILON, "Y coordinate failed for 0°");
  }

  @Test
  public void test90Degrees() {
    Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    Pose2d targetPose = new Pose2d(2, 3, new Rotation2d());
    Translation2d result = worldToRobotFrame(robotPose, targetPose);

    assertEquals(3, result.getX(), EPSILON, "X coordinate failed for 90°");
    assertEquals(-2.0, result.getY(), EPSILON, "Y coordinate failed for 90°");
  }

  @Test
  public void test45Degrees() {
    // Robot at (-2,-3) facing 45° (forward = (0.7071, 0.7071), right = (0.7071, -0.7071))
    Pose2d robotPose = new Pose2d(-2, -3, Rotation2d.fromDegrees(45));
    // Target at (0,0) => disp = (2,3)
    Pose2d targetPose = new Pose2d(0, 0, new Rotation2d());
    Translation2d result = worldToRobotFrame(robotPose, targetPose);
    // For 45°: sin(45°)=0.7071, cos(45°)=0.7071.
    // x_local = 2*0.7071 - 3*0.7071 ≈ -0.7071
    // y_local = 2*0.7071 + 3*0.7071 ≈ 3.5355
    assertEquals(-0.7071, result.getX(), 0.01, "X coordinate failed for 45°");
    assertEquals(3.5355, result.getY(), 0.01, "Y coordinate failed for 45°");
  }

  @Test
  public void testCustom() {
    // Custom case: Robot at (5,6) facing 180° (i.e. forward = (-1,0), right = (0,1))
    // Target at (0,0)
    Pose2d robotPose = new Pose2d(5, 6, Rotation2d.fromDegrees(180));
    Pose2d targetPose = new Pose2d(0, 0, new Rotation2d());
    Translation2d result = worldToRobotFrame(robotPose, targetPose);
    // disp = (0-5, 0-6) = (-5, -6)
    // For 180°: sin(pi)=0, cos(pi)=-1.
    // x_local = (-5)*0 - (-6)*(-1) = -6  (target is 6 units to the left)
    // y_local = (-5)*(-1) + (-6)*0 = 5   (target is 5 units forward)
    assertEquals(-6.0, result.getX(), EPSILON, "X coordinate failed for custom case");
    assertEquals(5.0, result.getY(), EPSILON, "Y coordinate failed for custom case");
  }
}