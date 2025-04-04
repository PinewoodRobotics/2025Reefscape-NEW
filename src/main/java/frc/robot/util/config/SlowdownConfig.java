package frc.robot.util.config;

import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public class SlowdownConfig {

  final double secondTierDistance;
  final double thirdTierDistance;

  final double firstTierMaxSpeedMultiplier;
  final double secondTierMaxSpeedMultiplier;
  final double thirdTierMaxSpeedMultiplier;
}
