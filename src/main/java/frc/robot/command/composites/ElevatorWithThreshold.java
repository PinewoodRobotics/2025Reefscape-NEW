package frc.robot.command.composites;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.command.driving.OdomAssistedTagAlignment;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ConditionCommand;
import frc.robot.util.config.WristElevatorConfig;

public class ElevatorWithThreshold extends ParallelCommandGroup {
  
  public ElevatorWithThreshold(
    SwerveSubsystem swerveSubsystem,
    OdometrySubsystem odometrySubsystem,
    ElevatorSubsystem elevatorSubsystem,
    WristElevatorConfig config
  ) {
    addCommands(
      new OdomAssistedTagAlignment(swerveSubsystem, odometrySubsystem, null, null, null, null, isFinished(), isScheduled()),
      new ConditionCommand(new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true),
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
