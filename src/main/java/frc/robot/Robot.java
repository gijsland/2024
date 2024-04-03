// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Thrower;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {
  /**
   * The drivetrain instance
   **/
  private Drivetrain drive;
  /**
   * The intake instance
   **/
  private Intake intake;
  /**
   * The arm instance
   **/
  private Arm arm;
  /**
   * The thrower instance
   **/
  private Thrower thrower;

  // private final Candle candle = new Candle();

  // The input Xbox controller
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  private SysIdRoutine sysIdRoutine;

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  ChoreoTrajectory traject;
  Field2d hmm;

  /**
   * Initialize the robot and prepare it for operation.
   **/
  @Override
  public void robotInit() {
    traject = Choreo.getTrajectory("cheese");

    hmm = new Field2d();
    SmartDashboard.putData("test field", hmm);

    Constants.UpdateSettings();

    DriverStation.silenceJoystickConnectionWarning(true);
    // this.candle.setAnimation(Candle.AnimationTypes.Fire);

    // this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    // this.candleSystem = Candle.getInstance();

    // this.candle.setAnimation(Candle.AnimationTypes.Larson);

    // Eject everything
    this.controller.leftBumper()
        .onTrue(this.intake.reverseIntake().alongWith(this.thrower.setLauncherVoltage(12.0))
            .andThen(this.thrower.setFeederVoltage(12.0)))
        .onFalse(this.intake.off().alongWith(this.thrower.off()));

    // Reset gyro
    this.controller.back().onTrue(this.drive.resetGyro());

    switch (Constants.SYSID_MODE) {
      default:
      case Teleop:
        // Make the robot drive in Teleoperated mode by default
        // this.drive.setDefaultCommand(this.drive.driverControl(
        //     this.controller.getHID()::getLeftY,
        //     this.controller.getHID()::getLeftX,
        //     this.controller.getHID()::getRightX));

        // Prepare underhand throw
        this.controller.a().onTrue(
            this.arm.setUnderhand()
                .alongWith(this.thrower.prepareSpeaker())
                .alongWith(this.intake.off())
                .alongWith(this.drive.setDriveMultipliers(
                    Constants.Drivetrain.Limits.UNDERHAND_LIN,
                    Constants.Drivetrain.Limits.UNDERHAND_ROT)));

        // Prepare overhand throw
        this.controller.y().onTrue(
            this.arm.setOverhand()
                .alongWith(this.thrower.prepareSpeaker())
                .alongWith(this.intake.off())
                .alongWith(this.drive.setDriveMultipliers(
                    Constants.Drivetrain.Limits.OVERHAND_LIN,
                    Constants.Drivetrain.Limits.OVERHAND_ROT)));

        // Prepare amp throw
        this.controller.b().onTrue(
            this.arm.setAmp()
                .alongWith(this.thrower.prepareAmp())
                .alongWith(this.intake.off())
                .alongWith(this.drive.setDriveMultipliers(
                    Constants.Drivetrain.Limits.AMP_LIN,
                    Constants.Drivetrain.Limits.AMP_ROT)));

        // Stow TODO: Maybe not turn the intake off?
        this.controller.x().onTrue(
            this.arm.setStow()
                .alongWith(this.intake.off())
                .alongWith(this.thrower.off())
                .alongWith(this.drive.setDriveMultipliers(
                    Constants.Drivetrain.Limits.NORMAL_LIN,
                    Constants.Drivetrain.Limits.NORMAL_ROT)));

        // Intake Note
        this.controller.rightBumper()
            .onTrue(
                this.intake.intakeNote()
                    .alongWith(this.arm.setIntake())
                    .alongWith(this.thrower.setIntake())
                    .alongWith(this.drive.setDriveMultipliers(
                        Constants.Drivetrain.Limits.INTAKE_LIN,
                        Constants.Drivetrain.Limits.INTAKE_ROT)))
            .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.off()));

        // Throw note
        this.controller.rightTrigger()
            .onTrue(this.thrower.launch())
            .onFalse(new SequentialCommandGroup(
                this.thrower.launch(),
                new WaitCommand(0.25),
                this.arm.setStow().alongWith(this.thrower.off())));

        // Toggle field-relative drive
        this.controller.start().onTrue(this.drive.toggleFieldRelative());

        break;
      case RotationSysId:
        this.setSysIdRoutine(
            2 * Math.PI, 4, 2 * Math.PI, 15,
            this.drive::rotateSysId, this.drive::getRotationalVelocity, "radians",
            () -> Units.degreesToRadians(this.drive.getGyroAngle()), "radians", this.drive);
        break;
      case StraightSysId:
        this.setSysIdRoutine(
            6, 10, 3, 15,
            this.drive::straightSysId, this.drive::getLinearVelocity, "meters", this.drive::getLinearDistanceMeters,
            "radians", this.drive);
        break;
      // case ThrowerRollerSysId:
      // this.setSysIdRoutine(
      //
      // )
    }

    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) {
      SignalLogger.setPath("/media/sda1/");
      this.controller.a().whileTrue(this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
          .onFalse(this.drive.driverControl(() -> 0.0, () -> 0.0, () -> 0.0));
      this.controller.y().whileTrue(this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
          .onFalse(this.drive.driverControl(() -> 0.0, () -> 0.0, () -> 0.0));
      this.controller.x().whileTrue(this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
          .onFalse(this.drive.driverControl(() -> 0.0, () -> 0.0, () -> 0.0));
      this.controller.b().whileTrue(this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
          .onFalse(this.drive.driverControl(() -> 0.0, () -> 0.0, () -> 0.0));
    }

    (new Trigger(() -> this.thrower.getFeederCurrent() >= Constants.INTAKE_NOTIFY_CURRENT && this.isTeleop()))
        .onTrue(new InstantCommand(() -> this.controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.6)))
        .onFalse(new InstantCommand(() -> this.controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
  }

  private void setSysIdRoutine(
      double rateMagnitude, double rateSeconds, double stepMagnitude, double timeoutSeconds,
      Consumer<Measure<Voltage>> sysIdFunction, Supplier<Double> velocitySupplier, String velocityUnits,
      Supplier<Double> positionSupplier, String positionUnits, Subsystem subsystem) {
    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(rateMagnitude).per(Seconds.of(rateSeconds)), Volts.of(stepMagnitude),
            Seconds.of(timeoutSeconds), state -> {
              SignalLogger.writeString("state", state.toString());
              SmartDashboard.putString("state", state.toString());
            }),
        new SysIdRoutine.Mechanism(sysIdFunction, l -> {
          SignalLogger.writeDouble("velocity", velocitySupplier.get(), velocityUnits);
          SignalLogger.writeDouble("position", positionSupplier.get(), positionUnits);
        }, subsystem));
  }

  /**
   * Routinely execute the currently scheduled command.
   **/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("** chassis speed", this.drive.getChassisSpeeds().toString());
    SmartDashboard.putString("** chassis position", this.drive.getPosition().toString());
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
  }

  @Override
  public void disabledInit() {
    this.controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop)
      SignalLogger.stop();
  }

  /**
   * Routinely update settings while disabled. This fixes the issue from 2023
   * where the autonomous chooser would not
   * properly update.
   **/
  @Override
  public void disabledPeriodic() {
    // Constants.UpdateSettings();
    // if (
    // !DriverStation.isJoystickConnected(Constants.Ports.INPUT_CONTROLLER)
    // || Math.abs(this.arm.getAngle() + 1.57) < Constants.Arm.AIM_ERROR
    // // || Math.abs(this.drive.getGyroAngle() -
    // Constants.STARTING_POSITION.getRotation().getDegrees()) < 20.0
    // ) {
    // // this.candle.setAnimation(Candle.AnimationTypes.Error);
    // } else {
    // // this.candle.setAnimation(Candle.AnimationTypes.Rainbow);
    // }

    // SmartDashboard.putString("Current Auto",
    // this.auto.getAutoName(this.auto.getSelectedAutoId()));
  }

  double autoStartTime;

  /**
   * Prepare autonomous mode.
   **/
  @Override
  public void autonomousInit() {
    autoStartTime = Timer.getFPGATimestamp();
    Constants.UpdateSettings();
    CommandScheduler.getInstance().cancelAll();
    ChoreoTrajectoryState oof = traject.sample(0);
    this.drive.resetPosition(oof.getPose());
  }

  private final ChoreoControlFunction choreoPID = Choreo.choreoSwerveController(
    new PIDController(10.0, 0.0, 0.2),
    new PIDController(10.0, 0.0, 0.2),
    new PIDController(2.0, 0.0, 0.0)
  );

  @Override
  public void autonomousPeriodic() {
    if (Timer.getFPGATimestamp() < autoStartTime + traject.getTotalTime()) {
      ChoreoTrajectoryState oof = traject.sample(Timer.getFPGATimestamp() - autoStartTime);
      hmm.setRobotPose(oof.getPose());
      

      ChassisSpeeds maybe = choreoPID.apply(this.drive.getPosition(), oof);

      this.drive.swerveDrive(maybe);
    } else {
      this.drive.swerveDrive(new ChassisSpeeds());
    }

    
  }

  double teleopStartTime;

  /**
   * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
    teleopStartTime = Timer.getFPGATimestamp();
    Constants.UpdateSettings();
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop)
      SignalLogger.start();
    if (Constants.alliance.isPresent() && Constants.alliance.get().equals(DriverStation.Alliance.Red)) {
      // Red alliance
      // this.candle.setColor(255,0,0);
    } else {
      // Blue alliance
      // this.candle.setColor(0,0,255);
    }
  }

  @Override
  public void teleopPeriodic() {
    if (Timer.getFPGATimestamp() < teleopStartTime + traject.getTotalTime()) {
      ChoreoTrajectoryState oof = traject.sample(Timer.getFPGATimestamp() - teleopStartTime);
      hmm.setRobotPose(oof.getPose());
    }

  }

  @Override
  public void simulationInit() {
    this.autonomousInit();
  }

  @Override
  public void simulationPeriodic() {
    this.autonomousPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
