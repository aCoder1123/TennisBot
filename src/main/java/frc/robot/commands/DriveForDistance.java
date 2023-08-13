// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.PID.*;

/** An example command that uses an example subsystem. */
public class DriveForDistance extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_swerve;
  private double xSpeed;
  private double ySpeed;
  private final double driveXDistance;
  private final double driveYDistance;
  private final double driveDistance;
  private final double driveAngle;
  private Pose2d currentPos;
  private boolean dControllerOn = true;

  private PIDController m_dController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController m_xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController m_yController = new PIDController(kDriveP, kDriveI, kDriveD);
  // private PIDController m_rotController = new PIDController(kTurnP, kTurnI,
  // kTurnD);
  private double distanceDriven = 0;

  /**
   * 
   *
   * @param drivetrain The subsystem used by this command.
   * @param distance   the distance in meters to drive
   * @param angle      the angle relative to the x-axis to drive at
   */
  public DriveForDistance(Drivetrain drivetrain, double distance, double angle) {
    m_swerve = drivetrain;
    currentPos = drivetrain.getFieldPosition();
    driveXDistance = Math.sin(Math.toRadians(angle)) * distance;
    driveYDistance = Math.cos(Math.toRadians(angle)) * distance;
    driveDistance = distance;
    driveAngle = angle;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_dController.setTolerance(kPositionTolerance * 15);
    m_xController.setTolerance(kPositionTolerance);
    m_yController.setTolerance(kPositionTolerance);

    m_dController.setSetpoint(driveDistance);
    m_xController.setSetpoint(currentPos.getX() + driveXDistance);
    m_yController.setSetpoint(currentPos.getY() + driveYDistance);
  }

  @Override
  public void execute() {
    if (m_dController.atSetpoint()) {
      dControllerOn = false;
      m_dController.close();
    }
    distanceDriven = Math.sqrt(Math.pow(m_swerve.getFieldPosition().getX() - currentPos.getX(), 2)
        + Math.pow(m_swerve.getFieldPosition().getY() - currentPos.getY(), 2));
    xSpeed = dControllerOn ? m_dController.calculate(distanceDriven) * Math.sin(Math.toRadians(driveAngle))
        : m_xController.calculate(m_swerve.getFieldPosition().getX());
    ySpeed = dControllerOn ? m_dController.calculate(distanceDriven) * Math.cos(Math.toRadians(driveAngle))
        : m_yController.calculate(m_swerve.getFieldPosition().getY());
    m_swerve.drive(xSpeed, ySpeed, 0, true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (m_xController.atSetpoint() && m_yController.atSetpoint()) ? true : false;
  }
}
