package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  public LEDSubsystem(int port, int stripLength) {
    m_led = new AddressableLED(port);
    m_buffer = new AddressableLEDBuffer(stripLength);
    m_led.setLength(stripLength);
    m_led.start();

    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  public AddressableLEDBuffer getBuffer() {
    return this.m_buffer;
  }

  @Override
  public void periodic() {
    m_led.setData(m_buffer);
  }

  public void setData(AddressableLEDBuffer newBuff) {
    this.m_buffer = newBuff;
  }

  public void setProgress(double cur, double max) {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> cur / max);
    pattern.applyTo(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}