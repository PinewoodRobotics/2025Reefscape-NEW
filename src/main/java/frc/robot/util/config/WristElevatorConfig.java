package frc.robot.util.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class WristElevatorConfig {
  
  public final Distance elevatorHeight;
  public final Rotation2d coralWristAngle;
  public final Rotation2d algaeWristAngle;

  public WristElevatorConfig(
    Distance height,
    Rotation2d angle1,
    Rotation2d angle2
  ) {
    elevatorHeight = height;
    coralWristAngle = angle1;
    algaeWristAngle = angle2;
  }

}
