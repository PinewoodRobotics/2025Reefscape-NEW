package frc.robot.command.finals;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ConditionCommand;

public class AlignAndIntake extends ParallelCommandGroup {
  
  public AlignAndIntake(
    SwerveSubsystem swerveSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    CoralSubsystem coralSubsystem
  ) {
    addCommands(
      //move to position
      
      
    );
  }
}
