package frc.robot.command.composites;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.driving.OdomAssistedTagAlignment;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.TagConfig;
import frc.robot.util.config.WristElevatorConfig;

public class ElevatorWithThreshold extends SequentialCommandGroup {

  public ElevatorWithThreshold(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristElevatorConfig config,
      DriveConfig driveConfig,
      Pose2d pole,
      long maxTimeNoTagSeen) {
    addCommands(
        new OdomAssistedTagAlignment(
            swerveSubsystem,
            odometrySubsystem,
            pole,
            driveConfig,
            new TagConfig(
                maxTimeNoTagSeen,
                AprilTagSubsystem.closestTagCurrently(maxTimeNoTagSeen)),
            AlignmentConstants.kSlowdownConfig,
            true,
            false).withTimeout(3),
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true));
  }
}
