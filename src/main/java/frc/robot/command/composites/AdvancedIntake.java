package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.alignment_commands.AlignCoralStation;
import frc.robot.command.coral_commands.CoralIntake;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AdvancedIntake extends ParallelCommandGroup {
  
  public AdvancedIntake(
    SwerveSubsystem swerveSubsystem,
    CoralSubsystem coralSubsystem,
    SwerveMoveTeleop moveCommand
  ) {
    addCommands(
      new AlignCoralStation(swerveSubsystem, moveCommand),
      new CoralIntake(coralSubsystem)
    );
  }
}
