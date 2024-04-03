package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import lib.swerve.SwerveKinematicLimits;
import lib.swerve.SwerveSetpoint;
import lib.swerve.SwerveSetpointGenerator;

import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  // Set the maximum movement limits for robot kinematics
  private final SwerveKinematicLimits limits = new SwerveKinematicLimits(Constants.Drivetrain.LINEAR_VELOCITY,
      Constants.Drivetrain.ACCELERATION, Constants.Drivetrain.ROTATION_VELOCITY);

  public final SwerveDriveKinematics swerveKinematics;

  private final SwerveSetpointGenerator swerveStateGenerator;
  private final SwerveDrivePoseEstimator positionEstimator;
  private SwerveSetpoint prevSetpoint;
  private boolean fieldRelative = true;
  private double linearMultiplier = 1.0;
  private double rotationalMultiplier = 1.0;
  private Field2d dashboardField = new Field2d();
  // private Field2d targetPositionField = new Field2d();
  // private Pose2d targetPosition = new Pose2d(0.0, 0.0, new Rotation2d());
  // private PIDController turnJoyPID = new
  // PIDController(Constants.Drivetrain.TURN_JOY_P,
  // Constants.Drivetrain.TURN_JOY_I, Constants.Drivetrain.TURN_JOY_D,
  // Constants.LOOP_INTERVAL);

  /**
   * Returns an initialized class of Drivetrain if one exists, or create a new one
   * if it doesn't (and return it).
   *
   * @return The drivetrain
   **/
  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  /**
   * The drivetrain subsystem for robot movement
   **/
  private Drivetrain() {
    this.gyro.calibrate();

    // Create the SwerveDriveKinematics object
    this.swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.FRONT_OFFSET, Constants.LEFT_OFFSET), // Front Left
        new Translation2d(Constants.FRONT_OFFSET, Constants.RIGHT_OFFSET), // Front Right
        new Translation2d(Constants.BACK_OFFSET, Constants.LEFT_OFFSET), // Back Left
        new Translation2d(Constants.BACK_OFFSET, Constants.RIGHT_OFFSET) // Back Right
    );

    this.swerveStateGenerator = new SwerveSetpointGenerator(this.swerveKinematics);

    // Initialize swerve modules
    this.frontLeft = new SwerveModule(
        "front left",
        Constants.Ports.FRONT_LEFT_DRIVE,
        Constants.Ports.FRONT_LEFT_TURN,
        Constants.Drivetrain.FF.FLS,
        Constants.Drivetrain.FF.FLV,
        Constants.Drivetrain.FF.FLA,
        true);
    this.frontRight = new SwerveModule(
        "front right",
        Constants.Ports.FRONT_RIGHT_DRIVE,
        Constants.Ports.FRONT_RIGHT_TURN,
        Constants.Drivetrain.FF.FRS,
        Constants.Drivetrain.FF.FRV,
        Constants.Drivetrain.FF.FRA,
        false);
    this.backLeft = new SwerveModule(
        "back left",
        Constants.Ports.BACK_LEFT_DRIVE,
        Constants.Ports.BACK_LEFT_TURN,
        Constants.Drivetrain.FF.BLS,
        Constants.Drivetrain.FF.BLV,
        Constants.Drivetrain.FF.BLA,
        true);
    this.backRight = new SwerveModule(
        "back right",
        Constants.Ports.BACK_RIGHT_DRIVE,
        Constants.Ports.BACK_RIGHT_TURN,
        Constants.Drivetrain.FF.BRS,
        Constants.Drivetrain.FF.BRV,
        Constants.Drivetrain.FF.BRA,
        false);

    // Set up position estimator (like SwerveDriveOdometry but with the ability to
    // add data from PhotonVision)
    this.positionEstimator = new SwerveDrivePoseEstimator(
        this.swerveKinematics,
        Rotation2d.fromDegrees(this.gyro.getAngle()),
        this.getPositionArray(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)) // Starting Position
    );

    // first previous module setpoint is wherever it boots up
    this.prevSetpoint = new SwerveSetpoint(this.getChassisSpeeds(),
        swerveKinematics.toSwerveModuleStates(this.getChassisSpeeds()));

    SmartDashboard.putData("Field", this.dashboardField);
  }

  /**
   * Routinely updates the robot's field-relative position according to motor
   * movement.
   **/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro angle", this.gyro.getAngle());
    SmartDashboard.putNumber("gyro velocity", this.getRotationalVelocity());

    this.positionEstimator.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        this.getPositionArray());

    Pose2d position = this.getPosition();
    this.dashboardField.setRobotPose(new Pose2d(position.getX(), position.getY(), position.getRotation()));
  }

  /**
   * Get the current angle of the robot
   *
   * @return The gyro angle in degrees
   **/
  public double getGyroAngle() {
    return this.gyro.getAngle();
  }

  /**
   * Creates a position array containing the current position states of each
   * swerve module.
   *
   * @return An array containing the swerve module positions.
   **/
  private SwerveModulePosition[] getPositionArray() {
    return new SwerveModulePosition[] {
        this.frontLeft.getPosition(),
        this.frontRight.getPosition(),
        this.backLeft.getPosition(),
        this.backRight.getPosition()
    };
  }

  /**
   * Reset the robot's field-relative position
   *
   * @param position The new offset/position of the robot
   **/
  public void resetPosition(Pose2d position) {
    this.resetGyro(position.getRotation().getDegrees());
    this.positionEstimator.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        this.getPositionArray(),
        position);

  }

  public Command waitUntilPositionCommand(double x, double y, double theta) {
    return new WaitUntilCommand(() -> {
      Pose2d position = this.getPosition();
      return Math.abs(position.getX() - x) < Constants.Drivetrain.WAIT_UNTIL_POSITION_ERROR.getX()
          && Math.abs(position.getY() - y) < Constants.Drivetrain.WAIT_UNTIL_POSITION_ERROR.getY()
          && Math.abs(position.getRotation().getDegrees() - theta)
              % (2 * Math.PI) < Constants.Drivetrain.WAIT_UNTIL_POSITION_ERROR.getRotation().getDegrees();
    });
  }

  /**
   * Returns the approximate field-relative position of the robot.
   *
   * @return A Pose2d containing the robot's position state
   **/
  public Pose2d getPosition() {
    return this.positionEstimator.getEstimatedPosition();
  }

  /**
   * Gets the combined speed of the robot.
   *
   * @return A ChassisSpeeds object containing information regarding the speed of
   *         the robot.
   **/
  public ChassisSpeeds getChassisSpeeds() {
    return this.swerveKinematics.toChassisSpeeds(
        this.frontLeft.getState(),
        this.frontRight.getState(),
        this.backLeft.getState(),
        this.backRight.getState());
  }

  public Command setDriveMultipliers(double linear, double rotational) {
    return this.runOnce(() -> {
      this.linearMultiplier = linear;
      this.rotationalMultiplier = rotational;
    });
  }

  /**
   * Reset the robot's orientation.
   *
   * @return A command that, when run, will reset the gyroscope's orientation.
   **/
  public Command resetGyro() {
    return this.resetGyro(0.0);
  }

  public Command resetGyro(double degrees) {
    return this.runOnce(() -> this.gyro.setGyroAngle(this.gyro.getYawAxis(), degrees));
  }

  public double getLinearVelocity() {
    return this.getChassisSpeeds().vxMetersPerSecond;
  }

  public double getRotationalVelocity() {
    return Math.toRadians(this.gyro.getRate());
  }

  public double getLinearDistanceMeters() {
    return this.positionEstimator.getEstimatedPosition().getX();
  }

  /**
   * Move the robot based on a provided ChassisSpeeds value
   *
   * @param movement The requested ChassisSpeeds
   **/
  public void swerveDrive(ChassisSpeeds movement) {
    movement = ChassisSpeeds.discretize(
        new ChassisSpeeds(movement.vxMetersPerSecond, movement.vyMetersPerSecond, movement.omegaRadiansPerSecond),
        Constants.LOOP_INTERVAL);

    SmartDashboard.putNumber("drive vx", movement.vxMetersPerSecond);
    SmartDashboard.putNumber("drive vy", movement.vyMetersPerSecond);
    SmartDashboard.putNumber("drive omega", movement.omegaRadiansPerSecond);

    this.prevSetpoint = this.swerveStateGenerator.generateSetpoint(this.limits, this.prevSetpoint, movement,
        Constants.LOOP_INTERVAL);

    // double NO_MOVEMENT = 0.01;
    // double NO_ROTATION = Math.toRadians(3.0);
    // if (Math.abs(movement.vxMetersPerSecond) <= NO_MOVEMENT &&
    // Math.abs(movement.vyMetersPerSecond) <= NO_MOVEMENT
    // && Math.abs(movement.omegaRadiansPerSecond) <= NO_ROTATION) {
    // // ignore super small inputs?
    // this.prevSetpoint = this.swerveStateGenerator.generateSetpoint(this.limits,
    // this.prevSetpoint,
    // new ChassisSpeeds(0.0, 0.0, 0.0), Constants.LOOP_INTERVAL);
    // } else {
    // this.prevSetpoint = this.swerveStateGenerator.generateSetpoint(this.limits,
    // this.prevSetpoint, movement,
    // Constants.LOOP_INTERVAL);
    // }

    SwerveModuleState[] moduleStates = this.prevSetpoint.mModuleStates;
    this.frontLeft.setTarget(moduleStates[0]);
    this.frontRight.setTarget(moduleStates[1]);
    this.backLeft.setTarget(moduleStates[2]);
    this.backRight.setTarget(moduleStates[3]);
  }

  public Command driverControl(Supplier<Double> getLeftJoyY, Supplier<Double> getLeftJoyX,
      Supplier<Double> getRightJoyX) {
    return this.run(() -> {
      // Remove low, fluctuating values from rotation input joystick
      double leftJoystickY = MathUtil.applyDeadband(getLeftJoyY.get(), Constants.Drivetrain.DRIVE_DEADBAND);
      double leftJoystickX = MathUtil.applyDeadband(getLeftJoyX.get(), Constants.Drivetrain.DRIVE_DEADBAND);
      double rightJoystickX = MathUtil.applyDeadband(getRightJoyX.get(), Constants.Drivetrain.TURN_DEADBAND);

      double vx = -leftJoystickY * Constants.Drivetrain.LINEAR_VELOCITY * this.linearMultiplier;
      double vy = -leftJoystickX * Constants.Drivetrain.LINEAR_VELOCITY * this.linearMultiplier;
      double omega = -rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_VELOCITY
          * this.rotationalMultiplier;

      if (this.fieldRelative) {
        this.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,
            Rotation2d.fromDegrees(this.getGyroAngle())));
      } else {
        this.swerveDrive(new ChassisSpeeds(vx, vy, omega));
      }
    });
  }

  public void rotateSysId(Measure<Voltage> vtheta) {
    double v = -vtheta.magnitude();
    SignalLogger.writeDouble("voltage", v);
    this.swerveDrive(new ChassisSpeeds(0.0, 0.0, v));
  }

  public void straightSysId(Measure<Voltage> vx) {
    double v = vx.magnitude();
    SignalLogger.writeDouble("voltage", v);
    this.swerveDrive(new ChassisSpeeds(v, 0.0, 0.0));
  }

  /**
   * Toggles field relative mode.
   *
   * @return A command that toggles field relative movement.
   **/
  public Command toggleFieldRelative() {
    return this.runOnce(() -> this.fieldRelative = !this.fieldRelative);
  }

  /**
   * Sets field relative
   *
   * @param fieldRelative The requested state of field relative
   **/
  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  /**
   * @return the state of field relative
   **/
  public boolean isFieldRelative() {
    return this.fieldRelative;
  }
}
