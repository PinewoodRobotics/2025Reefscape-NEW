package frc.robot.command.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import frc.robot.util.config.TagConfig;

public class TagTimedAlignment extends SequentialCommandGroup {

  public TagTimedAlignment(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      Pose2d targetPose,
      DriveConfig driveConfig,
      TagConfig tagConfig,
      SlowdownConfig slowdownConfig,
      double time,
      double speed) {
    addCommands(
        new OdomAssistedTagAlignment(
            swerveSubsystem,
            odometrySubsystem,
            targetPose,
            driveConfig,
            tagConfig,
            slowdownConfig,
            false,
            false),
        new MoveDirectionTimed(
            swerveSubsystem,
            speed,
            0,
            time));
  }
}
