package frc.robot.command.composites;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.algae_commands.AlgaeSetWristPosition;
import frc.robot.command.elevator_commands.SetElevatorHeight;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.config.WristElevatorConfig;

public class ElevatorAndAlgaeWrist extends SequentialCommandGroup {

    public ElevatorAndAlgaeWrist(
        ElevatorSubsystem elevatorSubsystem,
        AlgaeSubsystem algaeSubsystem,
        CoralSubsystem coralSubsystem,
        WristElevatorConfig config
    ) {
        addCommands(
            new SetElevatorHeight(elevatorSubsystem, config.elevatorHeight, true),
            new AlgaeSetWristPosition(algaeSubsystem, config.algaeWristAngle, true).asProxy(),
            new SetWristPosition(coralSubsystem, config.coralWristAngle, true).asProxy()
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}