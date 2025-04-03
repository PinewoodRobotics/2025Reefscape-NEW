package frc.robot.util.apriltags;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public class TimedTagPosition {

  final Pose2d pose;
  final int tagNumber;
  final long timestamp;
}
