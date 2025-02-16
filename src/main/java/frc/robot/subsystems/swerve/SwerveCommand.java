package frc.robot.subsystems.swerve;

public class SwerveCommand {

  /*
  private static SparkMax driveMotor;
  private static SparkMax turnMotor;
  
  public static void configureMotors() {
    driveMotor = new SparkMax(
        SwerveConstants.kFrontLeftDriveMotorPort,
        MotorType.kBrushless);
  
    turnMotor = new SparkMax(
        SwerveConstants.kFrontLeftTurningMotorPort,
        MotorType.kBrushless);
  
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(false)
        .smartCurrentLimit(SwerveConstants.kDriveCurrentLimit);
    driveConfig.encoder.velocityConversionFactor(
        (Math.PI * SwerveConstants.kWheelDiameterMeters) /
            (60 * SwerveConstants.kDriveGearRatio));
    driveMotor.configure(
        driveConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(false)
        .smartCurrentLimit(SwerveConstants.kTurnCurrentLimit);
    turnConfig.closedLoop
        .pid(
            SwerveConstants.kTurnP,
            SwerveConstants.kTurnI,
            SwerveConstants.kTurnD)
        .iZone(SwerveConstants.kTurnIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);
    turnConfig.encoder.positionConversionFactor(
        SwerveConstants.kTurnConversionFactor);
    turnMotor.configure(
        turnConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
  
  public static RunCommand getManualRunCommand(
      Swerve swerve,
      FlightStick flightStick_left,
      FlightStick flightStick_right) {
    return new RunCommand(
        () -> {
          swerve.drive(
              new Vec2(
                  CustomMath.deadband(
                      flightStick_right.getRawAxis(
                          FlightStick.AxisEnum.JOYSTICKY.value),
                      SwerveConstants.kXSpeedDeadband,
                      SwerveConstants.kXSpeedMinValue),
                  CustomMath.deadband(
                      flightStick_right.getRawAxis(
                          FlightStick.AxisEnum.JOYSTICKX.value)
                          *
                          -1,
                      SwerveConstants.kYSpeedDeadband,
                      SwerveConstants.kYSpeedMinValue)),
              CustomMath.deadband(
                  flightStick_right.getRawAxis(
                      FlightStick.AxisEnum.JOYSTICKROTATION.value)
                      *
                      -1,
                  SwerveConstants.kRotDeadband,
                  SwerveConstants.kRotMinValue),
              0.2);
  
          swerve.odometryTick();
        },
        swerve);
  }
  
  public static Runnable getCalibRunCommand(Swerve swerve, AHRSGyro gyro) {
    return () -> {
      System.out.println("null!!!");
      Communicator.subscribeAutobahn(
          "pos-extrapolator/robot-position",
          data -> {
            System.out.println("null");
            try {
              var position = RobotPosition.parseFrom(data);
              swerve.resetOdometryPosition(
                  new Pose2d(
                      -position.getEstimatedPosition()
                          .getPosition()
                          .getY(),
                      -position.getEstimatedPosition()
                          .getPosition()
                          .getX(),
                      new Rotation2d(
                          position.getEstimatedPosition()
                              .getDirection()
                              .getY(),
                          position.getEstimatedPosition()
                              .getDirection()
                              .getX())));
  
              double desiredAngle = Math.toDegrees(
                  Math.atan2(
                      position.getEstimatedPosition()
                          .getDirection()
                          .getY(),
                      position.getEstimatedPosition()
                          .getDirection()
                          .getX()));
              swerve.setGyroOffset(desiredAngle - gyro.getYaw());
              swerve.setCalibrated(true);
  
              gyro.setPositionAdjustment(
                  position.getEstimatedPosition().getPosition()
                      .getX(),
                  position.getEstimatedPosition().getPosition()
                      .getY(),
                  0);
              System.out.println("Reset Position to: " + position.toString());
            } catch (InvalidProtocolBufferException e) {
              e.printStackTrace();
            }
          });
    };
  }
  
  public static void unsubscribeCalib() {
    System.out.println("Stopped Subbing!");
    Communicator.unsubscribeAutobahn("pos-extrapolator/robot-position");
  }
     */
}
