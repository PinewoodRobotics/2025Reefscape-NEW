package frc.robot.util.apriltags;

import java.util.List;

public class TimedAprilTagPositions {

  public List<TagPosition> positions;
  public long timestamp;
  public String cameraName;

  public TimedAprilTagPositions(
    long timestamp,
    List<TagPosition> positions,
    String cameraName
  ) {
    this.positions = positions;
    this.timestamp = timestamp;
    this.cameraName = cameraName;
  }
}
