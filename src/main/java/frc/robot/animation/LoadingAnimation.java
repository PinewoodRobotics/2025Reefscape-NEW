package frc.robot.animation;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LoadingAnimation extends Command {
  private final LEDSubsystem ledSubsystem;
  private final int barSize;
  private int offset;

  public LoadingAnimation(LEDSubsystem ledSubsystem, int barSize) {
    this.ledSubsystem = ledSubsystem;
    this.barSize = barSize;
    addRequirements(ledSubsystem);
  }

  @Override
  public void execute() {
    AddressableLEDBuffer ledBuffer = ledSubsystem.getBuffer();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0); // Off
    }

    for (int i = 0; i < barSize; i++) {
      int pos = (offset + i) % ledBuffer.getLength();
      ledBuffer.setRGB(pos, 0, 255, 0); // Green bar
    }

    ledSubsystem.setData(ledBuffer);

    offset = (offset + 1) % ledBuffer.getLength();
  }
}
