package frc.robot.command.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.command.composites.ScoreCoral;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AlignAndScore extends SequentialCommandGroup {

  public AlignAndScore(
      SwerveSubsystem swerveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      WristElevatorConfig config) {
    addCommands(
        // new ElevatorWithThreshold(swerveSubsystem, elevatorSubsystem, config),
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, false),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 1500),
        new ScoreCoral(coralSubsystem, config),
        new MoveDirectionTimed(swerveSubsystem, 0.25, 0, 500),
        new SetElevatorHeight(elevatorSubsystem, ElevatorConstants.kDefaultHeight, false)
    //new SetWristPosition(coralSubsystem, CoralConstants.kDefaultAngle, false),
    //new SetElevatorHeight(elevatorSubsystem, ElevatorConstants.kMinHeight, false)
    );
  }
}
