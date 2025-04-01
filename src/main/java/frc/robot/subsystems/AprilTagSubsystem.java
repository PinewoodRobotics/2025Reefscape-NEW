package frc.robot.subsystems;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Communicator;
import java.util.ArrayList;
import java.util.List;
import proto.AprilTag.AprilTags;

public class AprilTagSubsystem extends SubsystemBase {

  public static TimedAprilTagPositions latestTagPositions;

  public static class TagPosition {

    public Pose2d pose;
    public int tagNumber;

    public TagPosition(Pose2d pose, int tagNumber) {
      this.pose = pose;
      this.tagNumber = tagNumber;
    }
  }

  public static class TimedAprilTagPositions {

    public List<TagPosition> positions;
    public long timestamp;

    public TimedAprilTagPositions(long timestamp, List<TagPosition> positions) {
      this.positions = positions;
      this.timestamp = timestamp;
    }
  }

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
      posePublishTopic,
      AprilTagSubsystem::onMessage
    );
  }

  private static double[][] fromFloatList(
    List<Float> flatList,
    int rows,
    int cols
  ) {
    if (flatList == null || flatList.size() != rows * cols) {
      throw new IllegalArgumentException(
        "The provided list does not match the specified dimensions."
      );
    }

    double[][] result = new double[rows][cols];
    double[] flatArray = flatList
      .stream()
      .mapToDouble(Float::doubleValue)
      .toArray();

    for (int i = 0; i < rows; i++) {
      System.arraycopy(flatArray, i * cols, result[i], 0, cols);
    }
    return result;
  }

  private static void onMessage(byte[] data) {
    try {
      var aprilTags = AprilTags.parseFrom(data);
      List<TagPosition> tagPositions = new ArrayList<>();

      for (var tag : aprilTags.getTagsList()) {
        var position_t = tag.getPoseTList();
        var rotation_R = fromFloatList(tag.getPoseRList(), 3, 3);

        var x = position_t.get(0);
        var z = position_t.get(2);

        tagPositions.add(
          new TagPosition(
            new Pose2d(
              x,
              z,
              new Rotation2d(-rotation_R[2][0], -rotation_R[2][2])
            ),
            tag.getTagId()
          )
        );
      }

      AprilTagSubsystem.latestTagPositions =
        new TimedAprilTagPositions(aprilTags.getTimestamp(), tagPositions);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    if (latestTagPositions != null) {
      // System.out.println(latestTagPositions.positions.get(0).pose);
    }
  }
}
