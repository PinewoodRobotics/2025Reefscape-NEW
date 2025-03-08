package frc.robot.subsystems;

import com.google.protobuf.InvalidProtocolBufferException;

import frc.robot.util.Communicator;
import proto.lidar.Lidar.PointCloud2d;

public class LidarSubsystem {
  private static PointCloud2d lastPointCloud;
  private static long lastTimePoint;
  private static String subTopic;

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
        posePublishTopic,
        LidarSubsystem::onMessage);
    LidarSubsystem.subTopic = posePublishTopic;
  }

  public static PointCloud2d getPtCloud() {
    return LidarSubsystem.lastPointCloud;
  }

  public static long getTime() {
    return LidarSubsystem.lastTimePoint;
  }

  private static void onMessage(byte[] data) {
    try {
      LidarSubsystem.lastTimePoint = System.currentTimeMillis();
      LidarSubsystem.lastPointCloud = PointCloud2d.parseFrom(data);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
