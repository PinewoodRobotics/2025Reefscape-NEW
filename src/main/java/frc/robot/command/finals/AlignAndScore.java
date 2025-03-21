package frc.robot.command.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.AutoGoToAprilTag;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.command.composites.ElevatorWithThreshold;
import frc.robot.command.composites.ScoreCoral;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AlignAndScore extends SequentialCommandGroup {
  
  public AlignAndScore(
    SwerveSubsystem swerveSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    CoralSubsystem coralSubsystem,
    VisionSubsystem visionSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      // new ElevatorWithThreshold(swerveSubsystem, elevatorSubsystem, config),
      new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, false),
      new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 1500),
      new AutoGoToAprilTag(swerveSubsystem, visionSubsystem, 0, 1, 0)
      //new ScoreCoral(coralSubsystem, config),
      //new SetWristPosition(coralSubsystem, CoralConstants.kDefaultAngle, false),
      //new SetElevatorHeight(elevatorSubsystem, ElevatorConstants.kMinHeight, false)
    );
  }
}
