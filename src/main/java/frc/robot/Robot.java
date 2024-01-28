// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Drivetrain drive;
  CommandXboxController xb = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  @Override
  public void robotInit() {
    Constants.UpdateSettings();
    this.drive = Drivetrain.getInstance();

    this.drive.setDefaultCommand(Commands.run(() -> drive.swerveDrive(
      xb.getLeftY() * Constants.Drivetrain.FORWARD_METERS_PER_SECOND * -1,
      xb.getLeftX() * Constants.Drivetrain.HORIZONTAL_METERS_PER_SECOND * -1,
      xb.getRightX() * Math.abs(xb.getRightX()) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1),
      drive
    ));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

//  @Override
//  public void disabledInit() {}
//
//  @Override
//  public void disabledPeriodic() {}
//
//  @Override
//  public void autonomousInit() {}
//
//  @Override
//  public void autonomousPeriodic() {}
//
//  @Override
//  public void teleopInit() {}

//  @Override
//  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

//  @Override
//  public void testPeriodic() {}
}
