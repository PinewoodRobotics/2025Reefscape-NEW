package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.apriltags.TimedTagPosition;
import proto.AprilTag.AprilTags;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

public class AprilTagSubsystem extends SubsystemBase {

  private static Map<Integer, TimedTagPosition> latestTagPositions = Collections.synchronizedMap(
      new HashMap<Integer, TimedTagPosition>());
  private static Map<Integer, List<TimedTagPosition>> lastTagPositions = Collections.synchronizedMap(
      new HashMap<Integer, List<TimedTagPosition>>());
  private static final int timeCleanMs = 500;
  private static final int MAX_HISTORY_SIZE = 100; // Prevent unbounded growth
  private static final SimpleMatrix cameraOutputToRobotRotation = new SimpleMatrix(
      new double[][] { { 0, 0, 1 }, { -1, 0, 0 }, { 0, -1, 0 } });

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
        posePublishTopic,
        AprilTagSubsystem::onMessage);
  }

  public static List<TimedTagPosition> getLatestTagPositions() {
    synchronized (latestTagPositions) {
      List<TimedTagPosition> allTags = new ArrayList<>();
      for (var tagPosition : latestTagPositions.values()) {
        allTags.add(tagPosition);
      }

      return allTags;
    }
  }

  public static TimedTagPosition getLatestTagPosition(int tagId) {
    synchronized (latestTagPositions) {
      return latestTagPositions.get(tagId);
    }
  }

  public static List<TimedTagPosition> getTagPoseHistory(int tagId) {
    synchronized (lastTagPositions) {
      return lastTagPositions.get(tagId);
    }
  }

  private static void onMessage(byte[] data) {
    try {
      var aprilTags = AprilTags.parseFrom(data);
      List<TimedTagPosition> tagPositions = new ArrayList<>();

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
            new TimedTagPosition(
                CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot),
                tag.getTagId(),
                System.currentTimeMillis()));
      }

      synchronized (latestTagPositions) {
        for (var tagPosition : tagPositions) {
          latestTagPositions.remove(tagPosition.getTagNumber());
          latestTagPositions.put(tagPosition.getTagNumber(), tagPosition);

          // Initialize list if it doesn't exist
          lastTagPositions.computeIfAbsent(
              tagPosition.getTagNumber(),
              k -> new ArrayList<>());

          List<TimedTagPosition> tagHistory = lastTagPositions.get(
              tagPosition.getTagNumber());
          if (tagHistory != null) {
            // Remove old entries
            tagHistory.removeIf(tag -> System.currentTimeMillis() - tag.getTimestamp() > timeCleanMs);

            // Add new position
            tagHistory.add(tagPosition);

            // Prevent unbounded growth
            while (tagHistory.size() > MAX_HISTORY_SIZE) {
              tagHistory.remove(0); // Remove oldest entry
            }
          }
        }
      }

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
                              .setX((float) latestTagPositions.get(9).getPose().getX())
                              .setY((float) latestTagPositions.get(9).getPose().getY())
                              .build())
                      .setDirection(
                          Vector2
                              .newBuilder()
                              .setX(
                                  (float) latestTagPositions
                                      .get(9)
                                      .getPose()
                                      .getRotation()
                                      .getCos())
                              .setY(
                                  (float) latestTagPositions
                                      .get(9)
                                      .getPose()
                                      .getRotation()
                                      .getSin())
                              .build())
                      .build())
              .build()
              .toByteArray());
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
