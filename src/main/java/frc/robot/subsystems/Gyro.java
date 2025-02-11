package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IGyroscopeLike;
import frc.robot.util.Communicator;
import proto.ImuOuterClass.Imu;

/**
 * @apiNote publish() should be called periodically.
 */
public class Gyro extends SubsystemBase implements IGyroscopeLike {

  private final AHRS m_gyro;
  private final String pubTopic;

  public Gyro(I2C.Port i2c_port_id, String pubTopic) {
    this.m_gyro = new AHRS(i2c_port_id);
    this.pubTopic = pubTopic;
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public void publish() {
    Imu imu = Imu
      .newBuilder()
      .setYaw(m_gyro.getYaw())
      .setRoll(m_gyro.getRoll())
      .setPitch(m_gyro.getPitch())
      .setAccelerationX(m_gyro.getWorldLinearAccelX())
      .setAccelerationY(m_gyro.getWorldLinearAccelY())
      .setAccelerationZ(m_gyro.getWorldLinearAccelZ())
      .setX(m_gyro.getDisplacementX())
      .setY(m_gyro.getDisplacementY())
      .setZ(m_gyro.getDisplacementZ())
      .setTimestamp(System.currentTimeMillis())
      .build();

    Communicator.sendMessageAutobahn(pubTopic, imu.toByteArray());
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
}
