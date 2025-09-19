package frc.robot.constants;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.camera.CameraSystem;
import frc.robot.util.CustomMath;

public class PiConstants {
  public static class CameraConstants {

    public static final CameraSystem[] photonCamerasInUse = new CameraSystem[] {
        new CameraSystem(
            "Arducam_OV9281_USB_Camera", // left camera facing 45 deg inwards
            new Transform2d(0.33, 0.33, new Rotation2d(Math.toRadians(-45)))),
        new CameraSystem(
            "Arducam_OV9281_USB_Camera (1)", // right camera facing 45 deg inwards
            new Transform2d(0.33, -0.33, new Rotation2d(Math.toRadians(45)))),
    };
  }
}
