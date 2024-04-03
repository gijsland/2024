package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends Command {

    private ChoreoTrajectory trajectory;
    private ChoreoControlFunction controller;
    private Timer m_timer;
    private Drivetrain m_drive;

    public FollowPath(String name, Drivetrain drive) {
        m_drive = drive;
        addRequirements(drive);
        this.trajectory = Choreo.getTrajectory(name);
        this.controller = Choreo.choreoSwerveController(
                new PIDController(12.0, 0.0, 0.6),
                new PIDController(12.0, 0.0, 0.6),
                new PIDController(5.0, 0.0, 0.1));
        this.m_timer = new Timer();
    }

    @Override
    public void initialize() {
        this.m_timer.restart();
    }

    @Override
    public void execute() {
        ChoreoTrajectoryState state = this.trajectory.sample(this.m_timer.get());
        this.m_drive.swerveDrive(this.controller.apply(this.m_drive.getPosition(), state));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            this.m_drive.swerveDrive(new ChassisSpeeds());
        } else {
            this.m_drive.swerveDrive(trajectory.getFinalState().getChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        return this.m_timer.hasElapsed(this.trajectory.getTotalTime());
    }
}
