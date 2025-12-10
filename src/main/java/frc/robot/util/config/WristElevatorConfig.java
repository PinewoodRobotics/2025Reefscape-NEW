package frc.robot.util.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class WristElevatorConfig {
  
  public final Distance elevatorHeight;
  public final Rotation2d wristAngle;

  public WristElevatorConfig(
    Distance height,
    Rotation2d angle
  ) {
    elevatorHeight = height;
    wristAngle = angle;
  }

}
