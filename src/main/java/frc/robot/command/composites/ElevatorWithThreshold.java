package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.driving.OdomAssistedTagAlignment;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ConditionCommand;
import frc.robot.util.config.WristElevatorConfig;
import java.util.function.BooleanSupplier;

public class ElevatorWithThreshold extends ParallelCommandGroup {

  public ElevatorWithThreshold(
    SwerveSubsystem swerveSubsystem,
    OdometrySubsystem odometrySubsystem,
    ElevatorSubsystem elevatorSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      new ConditionCommand(
        new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true),
        new BooleanSupplier() {
          @Override
          public boolean getAsBoolean() {
            return true;
          }
        } //replace with closeToSetpoint
      )
    );
  }
}
