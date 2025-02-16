package frc.robot.hardware;

public interface IGyroscopeLike {
  public double[] getYPR();

  public double[] getLinearAccelerationXYZ();

  public double[] getAngularVelocityXYZ();

  public double[] getQuaternion();

  public double[] getLinearVelocityXYZ();

  public double[] getPoseXYZ();

  public void reset();

  public void setAngleAdjustment(double angle);

  public void setPositionAdjustment(double x, double y, double z);

  // Default implementations

  public default double getYaw() {
    var ypr = getYPR();
    return ypr[0];
  }

  public default double getPitch() {
    var ypr = getYPR();
    return ypr[1];
  }

  public default double getRoll() {
    var ypr = getYPR();
    return ypr[2];
  }

  public default double getLinearAccelerationX() {
    var accel = getLinearAccelerationXYZ();
    return accel[0];
  }

  public default double getLinearAccelerationY() {
    var accel = getLinearAccelerationXYZ();
    return accel[1];
  }

  public default double getLinearAccelerationZ() {
    var accel = getLinearAccelerationXYZ();
    return accel[2];
  }

  public default double getAngularVelocityX() {
    var vel = getAngularVelocityXYZ();
    return vel[0];
  }

  public default double getAngularVelocityY() {
    var vel = getAngularVelocityXYZ();
    return vel[1];
  }

  public default double getAngularVelocityZ() {
    var vel = getAngularVelocityXYZ();
    return vel[2];
  }

  public default double getQuaternionW() {
    var quat = getQuaternion();
    return quat[0];
  }

  public default double getQuaternionX() {
    var quat = getQuaternion();
    return quat[1];
  }

  public default double getQuaternionY() {
    var quat = getQuaternion();
    return quat[2];
  }

  public default double getQuaternionZ() {
    var quat = getQuaternion();
    return quat[3];
  }

  public default double getLinearVelocityX() {
    var vel = getLinearVelocityXYZ();
    return vel[0];
  }

  public default double getLinearVelocityY() {
    var vel = getLinearVelocityXYZ();
    return vel[1];
  }

  public default double getLinearVelocityZ() {
    var vel = getLinearVelocityXYZ();
    return vel[2];
  }

  public default double getPoseX() {
    var pose = getPoseXYZ();
    return pose[0];
  }

  public default double getPoseY() {
    var pose = getPoseXYZ();
    return pose[1];
  }

  public default double getPoseZ() {
    var pose = getPoseXYZ();
    return pose[2];
  }
}
