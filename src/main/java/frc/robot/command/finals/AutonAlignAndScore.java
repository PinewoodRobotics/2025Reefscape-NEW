package frc.robot.command.finals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AutonAlignAndScore extends SequentialCommandGroup {

  public AutonAlignAndScore(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      WristElevatorConfig config,
      Pose2d pole,
      long driveFor) {
    addCommands(
        new MoveDirectionTimed(swerveSubsystem, -0.3, 0, driveFor),
        new AssistedScore(
            swerveSubsystem,
            odometrySubsystem,
            elevatorSubsystem,
            coralSubsystem,
            config,
            pole));
  }
}
