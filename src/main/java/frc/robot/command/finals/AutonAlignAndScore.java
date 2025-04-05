package frc.robot.command.finals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import frc.robot.util.config.WristElevatorConfig;

public class AutonAlignAndScore extends SequentialCommandGroup {

  public AutonAlignAndScore(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      WristElevatorConfig config,
      DriveConfig driveConfig,
      SlowdownConfig slowdownConfig,
      Pose2d pole,
      Pose2d odomDrivingPose,
      long driveFor,
      long timeNoTagSeen) {
    addCommands(
        new MoveDirectionTimed(swerveSubsystem, -0.3, 0, driveFor),
        new WaitCommand(driveFor / 1000),
        new AssistedScore(
            swerveSubsystem,
            odometrySubsystem,
            elevatorSubsystem,
            coralSubsystem,
            config,
            driveConfig,
            pole,
            timeNoTagSeen));
  }
}
