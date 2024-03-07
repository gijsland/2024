package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private static Launcher instance;
  private double targetAngle;  // Target radians for the arm to be moved to
  private double targetShooterVelocity;  // Velocity for rollers to spin at in rotations per second

  // Motor controllers
  private final TalonFX armMotor;  // Kraken for moving arm position
  private final TalonFX bottomShooterMotor;  // Kraken for bottom roller
  private final TalonFX topShooterMotor;  // Kraken for top roller
  private final TalonSRX feederMotor;  // Red motor

  private final MotionMagicExpoTorqueCurrentFOC armMotionMagic;
  private final VelocityTorqueCurrentFOC shooterController;

  private Launcher() {
    this.armMotor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);
    this.bottomShooterMotor = new TalonFX(Constants.Ports.BOTTOM_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.topShooterMotor = new TalonFX(Constants.Ports.TOP_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.feederMotor = new TalonSRX(Constants.Ports.ARM_FEEDER);

    // Configure gains for shooter to be used in the controller
    Slot0Configs shooterGains = new Slot0Configs()
            .withKP(Constants.Launcher.Shooter.P)
            .withKI(Constants.Launcher.Shooter.I)
            .withKD(Constants.Launcher.Shooter.D);

    // Limit current on shooter to avoid breaking motors
    TorqueCurrentConfigs shooterCurrentConfigs = new TorqueCurrentConfigs()
            .withPeakReverseTorqueCurrent(-Constants.Launcher.Shooter.MAX_CURRENT)
            .withPeakForwardTorqueCurrent(Constants.Launcher.Shooter.MAX_CURRENT);

    // Apply configuration values and orientation
    TalonFXConfiguration topShooterMotorConfig = new TalonFXConfiguration()
            .withTorqueCurrent(shooterCurrentConfigs)
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(shooterGains);
    TalonFXConfiguration bottomShooterMotorConfig = new TalonFXConfiguration()
            .withTorqueCurrent(shooterCurrentConfigs)
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(shooterGains);
    this.topShooterMotor.getConfigurator().apply(topShooterMotorConfig);
    this.bottomShooterMotor.getConfigurator().apply(bottomShooterMotorConfig);

    // Configure gains to be used in a velocity controller
    this.shooterController = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0)
            .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

    TorqueCurrentConfigs armCurrentLimits = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(Constants.Launcher.Arm.MAX_CURRENT)
            .withPeakReverseTorqueCurrent(-Constants.Launcher.Arm.MAX_CURRENT);
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.Launcher.Arm.CONVERSION_FACTOR);
    Slot0Configs armMotorGains = new Slot0Configs()
            .withKP(Constants.Launcher.Arm.P)
            .withKI(Constants.Launcher.Arm.I)
            .withKD(Constants.Launcher.Arm.D);
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration()
            .withTorqueCurrent(armCurrentLimits)
            .withFeedback(armFeedbackConfigs)
            .withSlot0(armMotorGains);
    this.armMotor.getConfigurator().apply(armMotorConfig);

    // Configure MotionMagic Object
//     other constructor: MotionMagicExpoTorqueCurrentFOC(double Position, double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
    this.armMotionMagic = new MotionMagicExpoTorqueCurrentFOC(Constants.Launcher.Arm.Positions.BASE)
            .withSlot(0)
            .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

    // Set Feeder Motor Limits
    TalonSRXConfiguration feederMotorConfigs = new TalonSRXConfiguration();
    feederMotorConfigs.peakCurrentLimit = Constants.Launcher.Feeder.PEAK_CURRENT_LIMIT;
    feederMotorConfigs.peakCurrentDuration = Constants.Launcher.Feeder.PEAK_CURRENT_DURATION;
    feederMotorConfigs.continuousCurrentLimit = Constants.Launcher.Feeder.CONTINUOUS_CURRENT_LIMIT;
    this.feederMotor.configAllSettings(feederMotorConfigs);
  }

  /**
    * Returns an initialized class of Launcher if one exists, or create a new one if it doesn't (and return it).
    * @return The launcher
   **/
  public static Launcher getInstance() {
    return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
  }

  @Override
  public void periodic(){
    this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle));

    this.topShooterMotor.setControl(this.shooterController.withVelocity(this.targetShooterVelocity));
    this.bottomShooterMotor.setControl(this.shooterController.withVelocity(this.targetShooterVelocity));

    // if the shooters are at shooting speed and the angle is reasonable, turn on feeder motor to give note to the launcher
//    if (shootersReady() && armReady()) {
//      this.feederMotor.setVoltage(0.0);
//    }
  }

  /**
    * Sets the target angle of the launcher.
    * @param angle The requested angle in radians.
   **/
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /**
    * Sets the target velocity of the launcher motors.
    * @param velocity The requested velocity in radians per second.
   **/
  public void setTargetShooterVelocity(double velocity) {
    this.targetShooterVelocity = velocity;
  }

  /**
    * Get the rotational velocity of the top launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getTopShooterVelocity() {
    return this.topShooterMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Get the rotational velocity of the bottom launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getBottomShooterVelocity() {
    return this.bottomShooterMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Check if the velocity of the launcher motors are greater than or equal to the target velocity.
    * @return if the shooters are ready.
   **/
  private boolean shootersReady() {
    return (this.getTopShooterVelocity() - this.targetShooterVelocity) <= Constants.Launcher.Shooter.VELOCITY_ERROR
      && (this.getBottomShooterVelocity() - this.targetShooterVelocity) <= Constants.Launcher.Shooter.VELOCITY_ERROR;
  }

  /**
    * Check if the arm has reached its target angle.
    * @return A boolean representing if the arm is ready.
   **/
  private boolean armReady() {
    return Math.abs(this.armMotor.getPosition().getValueAsDouble() * 2 * Math.PI - this.targetAngle) <= Constants.Launcher.Arm.POSITION_ERROR;
  }

  // Starts the feeder motor if it is ready
  public Command startFeederMotor() {
    return this.run(() -> {
      if (shootersReady() && armReady()) {
        this.feederMotor.set(TalonSRXControlMode.Current, Constants.Launcher.Feeder.TARGET_CURRENT);
      }
    });
  }

  // methods for each shot position for cleaner code in robot.java
  public void setIntake() {
    this.setTargetAngle(Constants.Launcher.Arm.Positions.INTAKE);
  }
  public void setOverhandShoot() {
    this.setTargetAngle(Constants.Launcher.Arm.Positions.OVERHAND_SHOOT);
  }
  public void setUnderhandShoot() {
    this.setTargetAngle(Constants.Launcher.Arm.Positions.UNDERHAND_SHOOT);
  }
  public void setAmp() {
    this.setTargetAngle(Constants.Launcher.Arm.Positions.AMP);
  }

  // methods for turning shooters in shooting direction and intaking direction
  public void intakeNote() {
    this.setTargetShooterVelocity(Constants.Launcher.Shooter.Velocities.INTAKE);
  }
  public void shootNote() {
    this.setTargetShooterVelocity(Constants.Launcher.Shooter.Velocities.SHOOT);
  }
}
