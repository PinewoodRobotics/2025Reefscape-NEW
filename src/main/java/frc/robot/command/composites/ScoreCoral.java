package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.coral_commands.CoralEject;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.util.ConditionCommand;
import frc.robot.util.boolean_suppliers.WristAtSetpoint;
import frc.robot.util.config.WristElevatorConfig;

public class ScoreCoral extends ParallelCommandGroup {
  
  public ScoreCoral(
    CoralSubsystem coralSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      new SetWristPosition(coralSubsystem, config.wristAngle, false).asProxy(),
      new ConditionCommand(new CoralEject(coralSubsystem).asProxy(), new WristAtSetpoint(coralSubsystem))
    );
  }
}
