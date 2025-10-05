package frc.robot.command.finals;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.MoveDirectionTimed;
import frc.robot.command.alignment_commands.AlignAndDriveForward;
import frc.robot.command.composites.ElevatorWithThreshold;
import frc.robot.command.composites.ScoreCoral;
import frc.robot.command.coral_commands.SetWristPosition;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AssistedScore extends SequentialCommandGroup {

  public AssistedScore(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      WristElevatorConfig config,
      Pose2d pole) {
    var aligned = new AtomicBoolean(false);
    var firstAttempt = new AlignAndDriveForward(pole, aligned);
    var retryAttempt = new SequentialCommandGroup(
        new MoveDirectionTimed(swerveSubsystem, 0.1, 0, 300, 0.5),
        new AlignAndDriveForward(pole, aligned));

    addCommands(
        firstAttempt
            .raceWith(new WaitCommand(10))
            .andThen(retryAttempt.onlyIf(() -> !aligned.get())),
        new ElevatorWithThreshold(
            swerveSubsystem,
            odometrySubsystem,
            elevatorSubsystem,
            config,
            pole),
        new ScoreCoral(coralSubsystem, config),
        new SetWristPosition(coralSubsystem, CoralConstants.kDefaultAngle, false));
  }
}
