package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.util.CustomMath;
import frc.robot.util.interfaces.IDataSubsystem;
import frc.robot.util.interfaces.IGyroscopeLike;
import proto.util.Position.Position3d;
import proto.util.Vector.Vector3;

public class AHRSGyro implements IGyroscopeLike, IDataSubsystem {

  private final AHRS m_gyro;
  private double xOffset = 0;
  private double yOffset = 0;
  private double zOffset = 0;

  public AHRSGyro(I2C.Port i2c_port_id) {
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
  public void setPositionAdjustment(double x, double y, double z) {
    xOffset = x;
    yOffset = y;
    zOffset = z;
    m_gyro.resetDisplacement();
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
        m_gyro.getDisplacementX() + xOffset,
        m_gyro.getDisplacementY() + yOffset,
        m_gyro.getDisplacementZ() + zOffset,
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
    return new byte[1];
  }

  @Override
  public String getPublishTopic() {
    return "robot/imu";
  }

  public Rotation2d getNoncontinuousAngle() {
    return Rotation2d.fromDegrees(CustomMath.wrapTo180(m_gyro.getAngle()));
  }
}
