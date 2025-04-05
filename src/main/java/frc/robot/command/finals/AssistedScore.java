package frc.robot.command.finals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.composites.ElevatorWithThreshold;
import frc.robot.command.composites.ScoreCoral;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.WristElevatorConfig;

public class AssistedScore extends SequentialCommandGroup {

  public AssistedScore(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      WristElevatorConfig config,
      DriveConfig driveConfig,
      Pose2d pole,
      long maxTimeNoTagSeen) {
    addCommands(
        new ElevatorWithThreshold(
            swerveSubsystem,
            odometrySubsystem,
            elevatorSubsystem,
            config,
            driveConfig,
            pole,
            maxTimeNoTagSeen),
        new ScoreCoral(coralSubsystem, config),
        new SetWristPosition(coralSubsystem, CoralConstants.kDefaultAngle, false),
        new SetElevatorHeight(
            elevatorSubsystem,
            ElevatorConstants.kDefaultHeight,
            false));
  }
}
