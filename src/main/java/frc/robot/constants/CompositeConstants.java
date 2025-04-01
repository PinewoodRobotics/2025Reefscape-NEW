package frc.robot.constants;

import frc.robot.util.config.WristElevatorConfig;

public class CompositeConstants {
  
  public static final WristElevatorConfig kL2 = new WristElevatorConfig(ElevatorConstants.kL2Height, CoralConstants.kL2Angle, AlgaeConstants.kFoldUpAngle);
  public static final WristElevatorConfig kL3 = new WristElevatorConfig(ElevatorConstants.kL3Height, CoralConstants.kL3Angle, AlgaeConstants.kFoldUpAngle);
  public static final WristElevatorConfig kL4 = new WristElevatorConfig(ElevatorConstants.kL4Height, CoralConstants.kL4Angle, AlgaeConstants.kFoldUpAngle);
  public static final WristElevatorConfig kAW2 = new WristElevatorConfig(ElevatorConstants.kL2Height, CoralConstants.outOfAlgaeWayAngle, AlgaeConstants.kL2Angle);
  public static final WristElevatorConfig kAW3 = new WristElevatorConfig(ElevatorConstants.kL3Height, CoralConstants.outOfAlgaeWayAngle, AlgaeConstants.kL3Angle);
  public static final WristElevatorConfig kAW4 = new WristElevatorConfig(ElevatorConstants.kL4Height, CoralConstants.outOfAlgaeWayAngle, AlgaeConstants.kL4Angle);
}
