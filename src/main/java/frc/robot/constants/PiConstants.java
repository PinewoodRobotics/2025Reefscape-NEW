package frc.robot.constants;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.CustomMath;

public class PiConstants {
  public static class CameraConstants {
    public static class CameraMount {
      public final PhotonCamera camera;
      public final Pose2d cameraToRobot;
      public final SimpleMatrix T_cameraInRobot;

      public CameraMount(String cameraName, Pose2d cameraInRobot) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraInRobot;
        this.T_cameraInRobot = CustomMath
            .fromPose2dToMatrix(cameraInRobot);
      }
    }

    public static final CameraMount[] photonCamerasInUse = new CameraMount[] {
        new CameraMount(
            "Arducam_OV9281_USB_Camera",
            new Pose2d(0.33, -0.33, new Rotation2d(Math.toRadians(-45)))),
    };
  }
}
