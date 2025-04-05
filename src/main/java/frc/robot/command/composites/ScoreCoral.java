package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.coral_commands.CoralEject;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class ScoreCoral extends SequentialCommandGroup {

  public ScoreCoral(CoralSubsystem coralSubsystem, WristElevatorConfig config) {
    addCommands(
        new SetWristPosition(coralSubsystem, config.wristAngle, true),
        new CoralEject(coralSubsystem));
  }
}
