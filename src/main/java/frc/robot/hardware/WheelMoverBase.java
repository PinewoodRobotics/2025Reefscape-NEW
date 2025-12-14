package frc.robot.hardware;

import org.pwrup.motor.WheelMover;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Common base for swerve modules in this project.
 *
 * We extend {@link WheelMover} because PWRUP's {@code SwerveDrive} /
 * {@code Wheel}
 * expects that type.
 */
public abstract class WheelMoverBase extends WheelMover {
  /** Module azimuth as a {@link Rotation2d}. */
  public abstract Rotation2d getRotation2d();

  /** Module distance + azimuth. */
  public abstract SwerveModulePosition getPosition();

  /** Module speed (m/s) + azimuth. */
  public abstract SwerveModuleState getState();

  /** Resets encoders to zero (or equivalent). */
  public abstract void reset();
}
