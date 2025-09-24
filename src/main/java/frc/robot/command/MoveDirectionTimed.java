package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveDirectionTimed extends Command {

  private final SwerveSubsystem m_swerveSubsystem;

  private int loopCount = 0;
  private final int maxLoops;

  private final double xSpeed, ySpeed;
  private final Vec2 direction;
  private final double modulo;

  public MoveDirectionTimed(SwerveSubsystem swerveSubsystem,
      double xSpeed,
      double ySpeed,
      double time) {
    this(swerveSubsystem, xSpeed, ySpeed, time, 0.5);
  }

  public MoveDirectionTimed(SwerveSubsystem swerveSubsystem,
      double xSpeed,
      double ySpeed,
      double time, double overalMultiplier) {
    m_swerveSubsystem = swerveSubsystem;

    maxLoops = (int) (time / 20); // FIXME: change this to 50
    // maxLoops = 20;

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.direction = new Vec2(xSpeed, ySpeed);
    this.modulo = overalMultiplier;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    loopCount = 0;
  }

  @Override
  public void execute() {
    // m_swerveSubsystem.drive(xSpeed, ySpeed, 0,
    // SwerveConstants.kAutonSpeedMultiplier);
    m_swerveSubsystem.driveRaw(
        direction,
        0,
        modulo);

    loopCount++;
  }

  @Override
  public void end(boolean interrupted) {
    // m_swerveSubsystem.drive(0, 0, 0, SwerveConstants.kAutonSpeedMultiplier);
    m_swerveSubsystem.driveRaw(
        new Vec2(0, 0),
        0,
        SwerveConstants.kAutonSpeedMultiplier);
  }

  @Override
  public boolean isFinished() {
    return loopCount >= maxLoops;
  }
}
