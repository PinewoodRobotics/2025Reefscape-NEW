package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ConditionCommand extends SequentialCommandGroup {
  
  public ConditionCommand(Command command, BooleanSupplier condition) {
    addCommands(
      new WaitUntilCommand(condition),
      command
    );
  }
}
