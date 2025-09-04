package frc.robot.subsystems.background;

import java.nio.ByteBuffer;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.util.RawFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import proto.sensor.Apriltags.AprilTagData;
import proto.sensor.Apriltags.WorldTags;
import proto.sensor.CameraSensor.ImageCompression;
import proto.sensor.CameraSensor.ImageData;
import proto.sensor.CameraSensor.ImageFormat;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;

public class CameraView extends SubsystemBase {
  public static CameraView self;

  public static CameraView GetInstance() {
    if (self == null) {
      self = new CameraView();
    }

    return self;
  }

  public CameraView() {
    Robot.communication.subscribe(PiConstants.AutobahnConfig.cameraViewTopic,
        NamedCallback.FromConsumer(this::subscription)).join();

    Robot.communication.subscribe(PiConstants.AutobahnConfig.cameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscribeTags)).join();
  }

  private void subscribeTags(byte[] data) {
    try {
      System.out.println("CAMERA");
      GeneralSensorData dat = GeneralSensorData.parseFrom(data);
      if (dat.getSensorName() != SensorName.APRIL_TAGS) {
        return;
      }

      AprilTagData tagData = dat.getApriltags();
      WorldTags worldTags = tagData.getWorldTags();
      var tagList = worldTags.getTagsList();
      var first = tagList.get(0);
      first.getPoseTList();
      var poseT = first.getPoseTList();
      Logger.recordOutput("tag/first", new Translation2d((double) poseT.get(0), (double) poseT.get(1)));
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private void subscription(byte[] data) {
    try {
      System.out.println("CAMERA");
      GeneralSensorData cameraViewMessage = GeneralSensorData.parseFrom(data);
      if (cameraViewMessage.getSensorName() != SensorName.CAMERA) {
        return;
      }

      // ImageData imageData = cameraViewMessage.getImage();
      /*
      if (imageData == null) {
        return;
      }
      
      Logger.recordOutput(cameraViewMessage.getSensorId() + "/frame", imageData.getImage().toByteArray());
       */
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private static ByteBuffer directBufferFrom(byte[] bytes) {
    ByteBuffer buf = ByteBuffer.allocateDirect(bytes.length);
    buf.put(bytes);
    buf.flip();
    return buf;
  }

  private RawFrame getRawFrame(ImageData imageData) {
    ImageFormat format = imageData.getFormat();
    ImageCompression compression = imageData.getCompression();

    int width = imageData.getWidth();
    int height = imageData.getHeight();

    byte[] bytes = imageData.getImage().toByteArray();
    ByteBuffer buf = directBufferFrom(bytes);

    RawFrame frame = new RawFrame();

    if (compression == ImageCompression.JPEG) {
      frame.setData(buf, width, height, 0, PixelFormat.kMJPEG);
      return frame;
    }

    if (compression == ImageCompression.NONE) {
      switch (format) {
        case BGR:
          frame.setData(buf, width, height, width * 3, PixelFormat.kBGR);
          return frame;

        case RGB:
          System.out.println("RGB provided; convert to BGR or pack to RGB565 before setData.");
          return null;

        case GRAY:
          frame.setData(buf, width, height, width, PixelFormat.kGray);
          return frame;

        case BGRA:
          frame.setData(buf, width, height, width * 4, PixelFormat.kBGRA);
          return frame;

        case RGBA:
          System.out.println("RGBA provided; convert to BGRA before setData.");
          return null;

        default:
          break;
      }
    }

    if (compression == ImageCompression.PNG) {
      System.out.println("PNG compression not supported for live frames; use JPEG or NONE.");
      return null;
    }

    System.out.println("Unsupported format: " + format + " with compression: " + compression);
    return null;
  }
}
