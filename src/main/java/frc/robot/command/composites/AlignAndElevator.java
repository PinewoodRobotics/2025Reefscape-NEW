package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AlignAndElevator extends ParallelCommandGroup {
  
  public AlignAndElevator(
    SwerveSubsystem swerveSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      //alignCommand
      new ElevatorWithThreshold(
        swerveSubsystem,
        elevatorSubsystem,
        config
        )
    );
  }
}
