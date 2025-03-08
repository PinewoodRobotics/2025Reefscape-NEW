package frc.robot.util.controller;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FlightStickNew extends Joystick {

  /**
   * @implNote the default button enum for the Flightstick
   */
  public enum ButtonEnum {
    // button indexes begin at 1 for some reason
    A(1),
    B(2),
    X(3),
    Y(4),
    B5(5),
    B6(6),
    B7(7),
    B8(8),
    LEFTSLIDERUP(9),
    LEFTSLIDERDOWN(10),
    RIGHTSLIDERUP(11),
    RIGHTSLIDERDOWN(12),
    MYSTERYBUTTON(13),
    MYSTERYBUTTON2(14),
    SCROLLPRESS(15),
    B16(16),
    B17(17),
    TRIGGER(18),
    B19(19),
    XBOX(20),
    SCREENSHARE(21),
    UPLOAD(22);

    public int value;

    /**
     * @param val for enum setting
     */
    ButtonEnum(int val) {
      this.value = val;
    }
  }

  /**
   * @apiNote this is for axis channels in a controller for the logitec
   */
  public enum AxisEnum {
    JOYSTICKX(0),
    JOYSTICKY(1),
    JOYSTICKROTATION(2),
    H2X(3),
    H2Y(4),
    LEFTSLIDER(5),
    SCROLLWHEEL(6),
    RIGHTSLIDER(7);

    public int value;
    
    /**
     * @param val for setting the port
     */
    AxisEnum(int val) {
      this.value = val;
    }
  }
  /**
   * @param port the port of the controller
   * @throws NoChannelFoundException if the channel is invalid that means that some code upstairs is buggy and needs to be fixed
   */
  public FlightStickNew(int port) {
    super(port);
  }

  public class Buttons {
    public JoystickButton A() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.A.value);
    }

    public JoystickButton B() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B.value);
    }

    public JoystickButton X() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.X.value);
    }

    public JoystickButton Y() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.Y.value);
    }

    public JoystickButton B5() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B5.value);
    }

    public JoystickButton B6() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B6.value);
    }

    public JoystickButton B7() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B7.value);
    }

    public JoystickButton B8() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B8.value);
    }

    public JoystickButton leftSliderUp() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.LEFTSLIDERUP.value);
    }

    public JoystickButton leftSliderDown() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.LEFTSLIDERDOWN.value);
    }

    public JoystickButton rightSliderUp() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.RIGHTSLIDERUP.value);
    }

    public JoystickButton rightSliderDown() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.RIGHTSLIDERDOWN.value);
    }

    public JoystickButton mysteryButton() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.MYSTERYBUTTON.value);
    }

    public JoystickButton mysteryButton2() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.MYSTERYBUTTON2.value);
    }

    public JoystickButton scrollPress() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.SCROLLPRESS.value);
    }

    public JoystickButton B16() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B16.value);
    }

    public JoystickButton B17() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B17.value);
    }

    public JoystickButton trigger() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.TRIGGER.value);
    }

    public JoystickButton B19() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.B19.value);
    }

    public JoystickButton XBOX() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.XBOX.value);
    }

    public JoystickButton screenshare() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.SCREENSHARE.value);
    }

    public JoystickButton upload() {
      return new JoystickButton(FlightStickNew.this, FlightStickNew.ButtonEnum.UPLOAD.value);
    }
  }

  public class Axis {
    public double getJoystickX() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.JOYSTICKX.value);
    }

    public double getJoystickY() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.JOYSTICKY.value);
    }

    public double getJoystickRotation() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.JOYSTICKROTATION.value);
    }

    public double getH2X() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.H2X.value);
    }

    public double getH2Y() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.H2Y.value);
    }

    public double getLeftSlider() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.LEFTSLIDER.value);
    }

    public double getScrollWheel() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.SCROLLWHEEL.value);
    }

    public double getRightSlider() {
      return FlightStickNew.this.getRawAxis(FlightStickNew.AxisEnum.RIGHTSLIDER.value);
    }
  }

}
