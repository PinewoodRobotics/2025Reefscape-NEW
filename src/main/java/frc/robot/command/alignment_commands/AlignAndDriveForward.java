package frc.robot.command.alignment_commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignAndDriveForward extends SequentialCommandGroup {

  private final SwerveSubsystem swerveSubsystem;

  public AlignAndDriveForward(Supplier<Pose2d> offset) {
    this.swerveSubsystem = SwerveSubsystem.GetInstance();
    addRequirements(swerveSubsystem);
    addCommands(
        new AlignTagNumber(offset),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 300));
  }
}
