// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.*;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(Drivetrain subsystem) {
    return Commands.sequence(new DriveForDistance(subsystem, 3, 60), new WaitCommand(1.0),
        new DriveForDistance(subsystem, -3, -90), new WaitCommand(1), new DriveForDistance(subsystem, -60, -3));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
