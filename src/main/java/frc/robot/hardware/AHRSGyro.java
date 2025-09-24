package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.util.CustomMath;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;
import proto.sensor.Imu.ImuData;
import proto.util.Position.Position3d;
import proto.util.Vector.Vector3;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class AHRSGyro implements IGyroscopeLike, IDataClass {
  private static AHRSGyro instance;

  private final AHRS m_gyro;
  private double xOffset = 0;
  private double yOffset = 0;
  private double zOffset = 0;

  public AHRSGyro(I2C.Port i2c_port_id) {
    this.m_gyro = new AHRS(i2c_port_id);
    m_gyro.reset();
  }

  public static AHRSGyro GetInstance() {
    if (instance == null) {
      instance = new AHRSGyro(I2C.Port.kMXP);
    }
    return instance;
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

  public Rotation2d getNoncontinuousAngle() {
    return Rotation2d.fromDegrees(CustomMath.wrapTo180(m_gyro.getAngle()));
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var poseXYZ = getPoseXYZ();
    var velocityXYZ = getLinearVelocityXYZ();
    var accelerationXYZ = getLinearAccelerationXYZ();
    var yaw = getYPR()[0];

    var position = Vector3.newBuilder()
        .setX((float) poseXYZ[0])
        .setY((float) poseXYZ[1])
        .setZ((float) poseXYZ[2]);

    var direction = Vector3.newBuilder()
        .setX((float) Math.cos(Math.toRadians(yaw)))
        .setY((float) Math.sin(Math.toRadians(yaw)))
        .setZ(0);

    var position2d = Position3d.newBuilder()
        .setPosition(position)
        .setDirection(direction);

    var velocity = Vector3.newBuilder()
        .setX((float) velocityXYZ[0])
        .setY((float) velocityXYZ[1])
        .setZ((float) velocityXYZ[2]);

    var acceleration = Vector3.newBuilder()
        .setX((float) accelerationXYZ[0])
        .setY((float) accelerationXYZ[1])
        .setZ((float) accelerationXYZ[2]);

    var imuData = ImuData.newBuilder()
        .setPosition(position2d)
        .setVelocity(velocity)
        .setAcceleration(acceleration);

    var all = GeneralSensorData.newBuilder().setImu(imuData).setSensorName(SensorName.IMU).setSensorId("imu")
        .setTimestamp(System.currentTimeMillis()).setProcessingTimeMs(0);

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "imu/imu";
  }
}
