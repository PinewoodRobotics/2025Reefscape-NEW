package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.algae_commands.SetAlgaeWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.ConditionCommand;
import frc.robot.util.boolean_suppliers.AlgaeWristClear;
import frc.robot.util.config.AlgaeElevatorConfig;

public class ElevatorAndAlgae extends ParallelCommandGroup {

  public ElevatorAndAlgae(
      ElevatorSubsystem elevatorSubsystem,
      AlgaeSubsystem algaeSubsystem,
      AlgaeElevatorConfig config) {
    addCommands(
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true),
        new ConditionCommand(
            new SetAlgaeWristPosition(algaeSubsystem, config.wristAngle, false).asProxy(),
            new AlgaeWristClear(elevatorSubsystem, config.elevatorHeight)));
  }
}
