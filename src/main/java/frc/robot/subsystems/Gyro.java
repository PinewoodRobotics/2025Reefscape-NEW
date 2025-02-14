package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IGyroscopeLike;
import frc.robot.util.Communicator;
import proto.ImuOuterClass.Imu;
import proto.util.Position.Position3d;
import proto.util.Vector.Vector3;

/**
 * @apiNote publish() should be called periodically.
 */
public class Gyro
  extends SubsystemBase
  implements IGyroscopeLike, IDataSubsystem {

  private final AHRS m_gyro;

  public Gyro(I2C.Port i2c_port_id) {
    this.m_gyro = new AHRS(i2c_port_id);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  @Override
  public double[] getYPR() {
    return new double[] {
      m_gyro.getYaw(),
      m_gyro.getPitch(),
      m_gyro.getRoll(),
    };
  }

  @Override
  public double[] getLinearAccelerationXYZ() {
    return new double[] {
      m_gyro.getWorldLinearAccelX(),
      m_gyro.getWorldLinearAccelY(),
      m_gyro.getWorldLinearAccelZ(),
    };
  }

  /**
   * @note not available on NavX
   */
  @Override
  public double[] getAngularVelocityXYZ() {
    return new double[] { 0, 0, 0 };
  }

  @Override
  public double[] getQuaternion() {
    return new double[] {
      m_gyro.getQuaternionW(),
      m_gyro.getQuaternionX(),
      m_gyro.getQuaternionY(),
      m_gyro.getQuaternionZ(),
    };
  }

  @Override
  public double[] getLinearVelocityXYZ() {
    return new double[] {
      m_gyro.getVelocityX(),
      m_gyro.getVelocityY(),
      m_gyro.getVelocityZ(),
    };
  }

  @Override
  public double[] getPoseXYZ() {
    return new double[] {
      m_gyro.getDisplacementX(),
      m_gyro.getDisplacementY(),
      m_gyro.getDisplacementZ(),
    };
  }

  @Override
  public void reset() {
    m_gyro.reset();
  }

  @Override
  public void setAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    return Imu
      .newBuilder()
      .setPosition(
        Position3d
          .newBuilder()
          .setPosition(
            Vector3
              .newBuilder()
              .setX(m_gyro.getDisplacementX())
              .setY(m_gyro.getDisplacementY())
              .setZ(m_gyro.getDisplacementZ())
              .build()
          )
          .setDirection(
            Vector3
              .newBuilder()
              .setX(m_gyro.getYaw())
              .setY(m_gyro.getPitch())
              .setZ(m_gyro.getRoll())
              .build()
          )
          .build()
      )
      .setVelocity(
        Vector3
          .newBuilder()
          .setX(m_gyro.getVelocityX())
          .setY(m_gyro.getVelocityY())
          .setZ(m_gyro.getVelocityZ())
          .build()
      )
      .setAcceleration(
        Vector3
          .newBuilder()
          .setX(m_gyro.getWorldLinearAccelX())
          .setY(m_gyro.getWorldLinearAccelY())
          .setZ(m_gyro.getWorldLinearAccelZ())
          .build()
      )
      .setTimestamp(System.currentTimeMillis())
      .build()
      .toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "robot/imu";
  }
}
