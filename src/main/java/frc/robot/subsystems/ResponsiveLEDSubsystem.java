package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ResponsiveLEDSubsystem extends SubsystemBase {
  private final CANdle candle;

  public ResponsiveLEDSubsystem(int channel) {
    this.candle = new CANdle(channel);
  }
}
