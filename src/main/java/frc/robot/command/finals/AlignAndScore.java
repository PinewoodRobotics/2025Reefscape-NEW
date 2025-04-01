package frc.robot.command.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignAndScore extends SequentialCommandGroup {

  /*
  public AlignAndScore(
      SwerveSubsystem swerveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      VisionSubsystem visionSubsystem,
      WristElevatorConfig config) {
    addCommands(
        // new ElevatorWithThreshold(swerveSubsystem, elevatorSubsystem, config),
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, false),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 1500),
        new AutoGoToAprilTag(swerveSubsystem, visionSubsystem, 0.32, -0.07, 0),
        new ScoreCoral(coralSubsystem, config),
        new MoveDirectionTimed(swerveSubsystem, 0.25, 0, 500),
        new SetElevatorHeight(elevatorSubsystem, ElevatorConstants.kDefaultHeight, false)
    //new SetWristPosition(coralSubsystem, CoralConstants.kDefaultAngle, false),
    //new SetElevatorHeight(elevatorSubsystem, ElevatorConstants.kMinHeight, false)
    );
  } */
}
