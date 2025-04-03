package frc.robot.util.config;

import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public class DriveConfig {

  final double translationStoppingDistance;
  final double angularStoppingDistanceDeg;
  final double maxRotationSpeed;
  final double maxTranslationSpeed;
}
