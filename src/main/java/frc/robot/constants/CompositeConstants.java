package frc.robot.constants;

import frc.robot.util.config.AlgaeElevatorConfig;
import frc.robot.util.config.WristElevatorConfig;

public class CompositeConstants {
  
  public static final WristElevatorConfig kL2 = new WristElevatorConfig(ElevatorConstants.kL2Height, CoralConstants.kL2Angle);
  public static final WristElevatorConfig kL3 = new WristElevatorConfig(ElevatorConstants.kL3Height, CoralConstants.kL3Angle);
  public static final WristElevatorConfig kL4 = new WristElevatorConfig(ElevatorConstants.kL4Height, CoralConstants.kL4Angle);

  public static final AlgaeElevatorConfig kMidAlgae = new AlgaeElevatorConfig(ElevatorConstants.kMidAlgaeHeight, AlgaeConstants.kIntakeAngle);
  public static final AlgaeElevatorConfig kHighAlgae = new AlgaeElevatorConfig(ElevatorConstants.kHighAlgaeHeight, AlgaeConstants.kIntakeAngle);
  public static final AlgaeElevatorConfig kBottom = new AlgaeElevatorConfig(ElevatorConstants.kIntakeHeight, AlgaeConstants.kWristDefaultAngle);
}
