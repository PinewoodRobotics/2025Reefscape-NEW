package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class ElevatorAndCoral extends SequentialCommandGroup {
  
  public ElevatorAndCoral(
    ElevatorSubsystem elevatorSubsystem,
    CoralSubsystem coralSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true),
      new SetWristPosition(coralSubsystem, config.wristAngle, true)
    );
  }
}
