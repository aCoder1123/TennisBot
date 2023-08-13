package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoysticks extends CommandBase {

    private Drivetrain m_swerve;
    private CommandXboxController m_controller;
    private boolean isFieldRelative;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /**
     * @param drivetrain    - current Drivetrain
     * @param fieldRelative - boolean
     * @param controller    - driver controller
     * 
     * 
     */

    public DriveWithJoysticks(Drivetrain drivetrain, boolean fieldRelative, CommandXboxController controller) {
        m_swerve = drivetrain;
        m_controller = controller;
        isFieldRelative = fieldRelative;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                * kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, isFieldRelative);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
