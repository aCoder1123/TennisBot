// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.PID.*;

public class Loader extends SubsystemBase {

    public Loader() {
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
