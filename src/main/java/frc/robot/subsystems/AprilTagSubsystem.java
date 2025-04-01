package frc.robot.subsystems;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.apriltags.TagPosition;
import frc.robot.util.apriltags.TimedAprilTagPositions;
import java.util.ArrayList;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import proto.AprilTag.AprilTags;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

public class AprilTagSubsystem extends SubsystemBase {

  private static List<TimedAprilTagPositions> latestTagPositions = new ArrayList<>();
  private static final SimpleMatrix cameraOutputToRobotRotation = new SimpleMatrix(
    new double[][] { { 0, 0, 1 }, { -1, 0, 0 }, { 0, -1, 0 } }
  );

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
      posePublishTopic,
      AprilTagSubsystem::onMessage
    );
  }

  public static List<TimedAprilTagPositions> getLatestTagPositions() {
    return AprilTagSubsystem.latestTagPositions;
  }

  private static void onMessage(byte[] data) {
    try {
      var aprilTags = AprilTags.parseFrom(data);
      List<TagPosition> tagPositions = new ArrayList<>();

      for (var tag : aprilTags.getTagsList()) {
        var positionT = CustomMath.fromFloatList(tag.getPoseTList(), 3, 1);
        var rotationR = CustomMath.fromFloatList(tag.getPoseRList(), 3, 3);

        var tagInCameraRotation =
          (
            cameraOutputToRobotRotation
              .mult(rotationR)
              .mult(cameraOutputToRobotRotation.transpose())
          );

        var tagInCameraPose = cameraOutputToRobotRotation.mult(positionT);
        var transformationMatrix = CustomMath.createTransformationMatrix(
          tagInCameraRotation,
          tagInCameraPose
        );
        var T_tagInCamera = CustomMath.from3dTransformationMatrixTo2d(
          transformationMatrix
        );

        var T_cameraInRobot = CameraConstants.cameras.get(
          aprilTags.getCameraName()
        );

        var T_tagInRobot = T_cameraInRobot.mult(T_tagInCamera);
        tagPositions.add(
          new TagPosition(
            CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot),
            tag.getTagId()
          )
        );
      }

      latestTagPositions.removeIf(timedTagPositions ->
        timedTagPositions.cameraName == aprilTags.getCameraName()
      );

      latestTagPositions.add(
        new TimedAprilTagPositions(
          aprilTags.getTimestamp(),
          tagPositions,
          aprilTags.getCameraName()
        )
      );

      Communicator.sendMessageAutobahn(
        "pos-extrapolator/robot-position",
        RobotPosition
          .newBuilder()
          .setEstimatedPosition(
            Position2d
              .newBuilder()
              .setPosition(
                Vector2
                  .newBuilder()
                  .setX((float) tagPositions.get(0).pose.getX())
                  .setY((float) tagPositions.get(0).pose.getY())
                  .build()
              )
              .setDirection(
                Vector2
                  .newBuilder()
                  .setX((float) tagPositions.get(0).pose.getRotation().getCos())
                  .setY((float) tagPositions.get(0).pose.getRotation().getSin())
                  .build()
              )
              .build()
          )
          .build()
          .toByteArray()
      );
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
