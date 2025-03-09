package frc.robot.util;

import frc.robot.constants.ElevatorConstants;

/**
 * @note MathFun = Math Functions
 * @apiNote this is the file where all of the math functions go
 */
public class MathFunc {

  /**
   * @param x the X position on the graph
   * @param yCoef the the height of the graph on the yAxis. This will increase / decrease the return val
   * @return The Y of the X on the graph
   */
  public static double getPosOnGraph(double x, double yCoef) {
    return Math.cbrt(x) + yCoef;
  }

  /**
   * @param values double values for which you want to get the biggest value
   * @return the max value form the input values
   */
  public static double max(double... values) {
    double m = values[0];

    for (double value : values) {
      if (value > m) {
        m = value;
      }
    }

    return m;
  }

  /**
   * @param values
   * @return the minimum value from the values
   */
  public static double min(double... values) {
    double m = values[0];
    for (double value : values) {
      if (value < m) {
        m = value;
      }
    }

    return m;
  }
  /**
   * Deadbands joystick input, then scales it from the deadband to 1. Ask Jared for clarification.
   * @param input the joystick input, [0, 1]
   * @param deadband ignores the input if it is less than this value, [0, 1]
   * @param minValue adds this value if the input overcomes the deadband, [0, 1]
   * @return the return value, [0, 1]
   */
  public static double deadband(
    double input,
    double deadband,
    double minValue
  ) {
    double output;
    double m = (1.0 - minValue) / (1.0 - deadband);

    if (Math.abs(input) < deadband) {
      output = 0;
    } else if (input > 0) {
      output = m * (input - deadband) + minValue;
    } else {
      output = m * (input + deadband) - minValue;
    }

    return output;
  }

  public static double plusMinusHalf(double in) {
    while (in > 0.5) {
      in -= 1;
    }
    while (in < -0.5) {
      in += 1;
    }
    return in;
  }

  public static double plusMinus180(double in) {
    while (in > 180) {
      in -= 360;
    }
    while (in < -180) {
      in += 360;
    }
    return in;
  }

  /**
   * For setpoint ramping, limits the change in setpoint by the maxRamp
   * 
   * @param setpoint Where you wants your setpoint to be
   * @param currentSetpoint Where your setpoint currently is
   * @param maxRamp How fast you want your setpoint to be able to change, in units / tick
   * @return The new current setpoint
   */
  public static double rampSetpoint(double setpoint, double currentSetpoint, double maxRamp) {
    if (setpoint - currentSetpoint > maxRamp) {
      return currentSetpoint += maxRamp;
    } else if (setpoint - currentSetpoint < -maxRamp) {
      return currentSetpoint -= maxRamp;
    }
    return currentSetpoint = setpoint;
  }
}
