package frc.robot.util.config;

import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public class TagConfig {

  final long maxTimeNoTagSeen;
  final int tagId;
}
