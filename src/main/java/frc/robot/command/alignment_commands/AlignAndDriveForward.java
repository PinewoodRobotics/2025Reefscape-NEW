package frc.robot.command.alignment_commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.concurrent.atomic.AtomicBoolean;

public class AlignAndDriveForward extends SequentialCommandGroup {

  private final SwerveSubsystem swerveSubsystem;

  public AlignAndDriveForward(Pose2d offset) {
    this(new Supplier<Pose2d>() {
      @Override
      public Pose2d get() {
        return offset;
      }
    });
  }

  public AlignAndDriveForward(Supplier<Pose2d> offset) {
    this.swerveSubsystem = SwerveSubsystem.GetInstance();
    addRequirements(swerveSubsystem);
    addCommands(
        new AlignTagNumber(offset),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 200, 0.1),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 250, 0.5));
  }

  /**
   * Overload that publishes whether alignment succeeded into the provided flag.
   * The flag will be set to true only if {@link AlignTagNumber} met its
   * thresholds. If the group is interrupted (e.g., by a timeout race), the flag
   * will remain unchanged.
   */
  public AlignAndDriveForward(Supplier<Pose2d> offset, AtomicBoolean alignmentSucceededFlag) {
    this.swerveSubsystem = SwerveSubsystem.GetInstance();
    addRequirements(swerveSubsystem);

    AlignTagNumber align = new AlignTagNumber(offset);
    addCommands(
        align,
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 200, 0.1),
        new MoveDirectionTimed(swerveSubsystem, -0.25, 0, 250, 0.5),
        new InstantCommand(() -> alignmentSucceededFlag.set(align.wasSuccessful())));
  }

  public AlignAndDriveForward(Pose2d offset, AtomicBoolean alignmentSucceededFlag) {
    this(() -> offset, alignmentSucceededFlag);
  }
}
