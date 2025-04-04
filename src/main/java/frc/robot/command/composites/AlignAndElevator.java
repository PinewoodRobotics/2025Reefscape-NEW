package frc.robot.command.composites;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class AlignAndElevator extends ParallelCommandGroup {
  
  public AlignAndElevator(
    SwerveSubsystem swerveSubsystem,
    OdometrySubsystem odometrySubsystem,
    ElevatorSubsystem elevatorSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      //alignCommand
      new ElevatorWithThreshold(
        swerveSubsystem,
        odometrySubsystem,
        elevatorSubsystem,
        config
        )
    );
  }
}
