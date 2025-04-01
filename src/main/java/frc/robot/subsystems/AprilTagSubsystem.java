package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.apriltags.TagPosition;
import frc.robot.util.apriltags.TimedAprilTagPositions;
import proto.AprilTag.AprilTags;

public class AprilTagSubsystem extends SubsystemBase {

  private static List<TimedAprilTagPositions> latestTagPositions = Collections.synchronizedList(
      new ArrayList<>());
  private static final SimpleMatrix cameraOutputToRobotRotation = new SimpleMatrix(
      new double[][] { { 0, 0, 1 }, { -1, 0, 0 }, { 0, -1, 0 } });

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
        posePublishTopic,
        AprilTagSubsystem::onMessage);
  }

  public static List<TimedAprilTagPositions> getLatestTagPositions() {
    synchronized (latestTagPositions) {
      return new ArrayList<>(latestTagPositions);
    }
  }

  private static void onMessage(byte[] data) {
    try {
      var aprilTags = AprilTags.parseFrom(data);
      List<TagPosition> tagPositions = new ArrayList<>();
      Pose2d robotInTag = new Pose2d();

      for (var tag : aprilTags.getTagsList()) {
        var positionT = CustomMath.fromFloatList(tag.getPoseTList(), 3, 1);
        var rotationR = CustomMath.fromFloatList(tag.getPoseRList(), 3, 3);

        var tagInCameraRotation = (cameraOutputToRobotRotation
            .mult(rotationR)
            .mult(cameraOutputToRobotRotation.transpose()));

        var tagInCameraPose = cameraOutputToRobotRotation.mult(positionT);
        var transformationMatrix = CustomMath.createTransformationMatrix(
            tagInCameraRotation,
            tagInCameraPose);
        var T_tagInCamera = CustomMath.from3dTransformationMatrixTo2d(
            transformationMatrix);

        var T_cameraInRobot = CameraConstants.cameras.get(
            aprilTags.getCameraName());

        var T_tagInRobot = T_cameraInRobot.mult(T_tagInCamera);
        tagPositions.add(
            new TagPosition(
                CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot),
                tag.getTagId()));

        robotInTag = CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot);
      }

      synchronized (latestTagPositions) {
        latestTagPositions
            .removeIf(timedTagPositions -> timedTagPositions.cameraName.equals(aprilTags.getCameraName()));

        latestTagPositions.add(
            new TimedAprilTagPositions(
                aprilTags.getTimestamp(),
                tagPositions,
                aprilTags.getCameraName()));
      }

      /*
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
                              .setX((float) robotInTag.getX())
                              .setY((float) robotInTag.getY())
                              .build())
                      .setDirection(
                          Vector2
                              .newBuilder()
                              .setX((float) robotInTag.getRotation().getCos())
                              .setY((float) robotInTag.getRotation().getSin())
                              .build())
                      .build())
              .build()
              .toByteArray()); */
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
