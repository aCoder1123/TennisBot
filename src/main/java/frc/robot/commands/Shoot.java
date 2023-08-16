// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Shoot extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Turret m_Turret;
    private final Loader m_Loader;
    private final Drivetrain m_Swerve;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Shoot(Turret turret, Loader loader, Drivetrain swerve) {
        m_Turret = turret;
        m_Loader = loader;
        m_Swerve = swerve;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret, loader, swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_Loader.isLoaded()) {
            throw new Error("Not Loaded");
            
        }
        m_Turret.setSearch(true);
        m_Turret.prepareShot(m_Swerve.getFieldPosition());
        m_Swerve.drive(0, 0, 0, true);
        m_Loader.setStandBy(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        while (!(m_Turret.shotReady() && m_Loader.isReady())) {
            m_Turret.prepareShot(m_Swerve.getFieldPosition());
            m_Loader.setStandBy(true);
        }
        m_Loader.loadBall();
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
