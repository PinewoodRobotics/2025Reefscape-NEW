package frc.robot.util;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

public class PinCommunication {

  private static DigitalOutput dio = new DigitalOutput(0);
  private static SPI spiPin = new SPI(SPI.Port.kOnboardCS1);

  public static void sendOnline() {
    dio.set(true);
  }
}
