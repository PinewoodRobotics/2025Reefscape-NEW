package frc.robot.command.composites;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.TagConfig;
import frc.robot.util.config.WristElevatorConfig;

public class ElevatorWithThreshold extends SequentialCommandGroup {

  public ElevatorWithThreshold(
      SwerveSubsystem swerveSubsystem,
      OdometrySubsystem odometrySubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristElevatorConfig config,
      Pose2d pole) {
    addCommands(
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true));
  }
}
