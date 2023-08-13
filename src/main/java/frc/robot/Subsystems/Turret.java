// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.PID.*;

public class Turret extends SubsystemBase {
  private PIDController m_RightFlywheelController = new PIDController(kFlywheelP, kFlywheelI, kFlywheelP);
  private PIDController m_LeftFlywheelController = new PIDController(kFlywheelP, kFlywheelI, kFlywheelP);
  private PIDController m_TurnController = new PIDController(kTurretP, kTurretI, kTurretD);
  private PIDController m_HoodController = new PIDController(kTurretP, kTurretI, kTurretD);

  private CANSparkMax m_LeftFlywheelMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_RightFlywheelMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_TurnMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_HoodMotor = new CANSparkMax(0, MotorType.kBrushless);

  private RelativeEncoder m_RightFlywheelEncoder;
  private RelativeEncoder m_LeftFlywheelEncoder;
  private RelativeEncoder m_TurnEncoder;
  private RelativeEncoder m_HoodEncoder;
  
  class TurretState {
      public double rpm = 0;
      public double turretRot = 0;
      public double hoodAngle = 0;
    }

  TurretState currentTurretState = new TurretState();
  TurretState desiredTurretState = new TurretState();
  
  private boolean onStandBy = true;
  
  public Turret() {
    currentTurretState.hoodAngle = 0;
    currentTurretState.turretRot = 0;

    m_TurnController.enableContinuousInput(0, 360);
    m_TurnController.setTolerance(0.5);
    m_HoodController.setTolerance(0.5);
    m_LeftFlywheelController.setTolerance(10);
    m_RightFlywheelController.setTolerance(10);

    m_RightFlywheelEncoder = m_RightFlywheelMotor.getEncoder();
    m_LeftFlywheelEncoder = m_LeftFlywheelMotor.getEncoder();
    m_TurnEncoder = m_TurnMotor.getEncoder();
    m_HoodEncoder = m_HoodMotor.getEncoder();
    
  }

  // public void setAsZero() {
    
  // }
  //todo empirically determine formula
  public double velocityToRPM(double velocity) {
    double rpm = (1 * Math.pow(velocity, 2) + 1* velocity + .1);
    return MathUtil.clamp(rpm, -1, 1);
  }

  public TurretState getTurretPos() {
    return currentTurretState;
  }

  public void setDesiredState(TurretState desiredPos) {
    desiredTurretState = desiredPos;
    desiredTurretState.hoodAngle = MathUtil.clamp(desiredPos.hoodAngle, 22.5, 45);
    m_TurnController.setSetpoint(desiredPos.turretRot);
    // m_HoodController.setSetpoint(desiredPos.turretRot - ((currentTurretState.hoodAngle - desiredPos.hoodAngle) * (24 / 18)));
    m_LeftFlywheelController.setSetpoint(desiredTurretState.rpm);
    m_RightFlywheelController.setSetpoint(desiredTurretState.rpm);
  }
  
  public TurretState getShotData(Pose2d robotPos) {
    TurretState shotData = new TurretState();
    return shotData;
  }

  //TODO math
  public void prepareShot()  {

  }

  public Object shotReady() {
    if (m_HoodController.atSetpoint() && m_TurnController.atSetpoint() && m_RightFlywheelController.atSetpoint() && m_LeftFlywheelController.atSetpoint()) {
      currentTurretState = desiredTurretState;
      return desiredTurretState;
    }
    return null;
  }

  //TODO implement flywheel feed forward
  @Override
  public void periodic() {
    if (onStandBy) {
      m_RightFlywheelMotor.set(MathUtil.clamp(m_RightFlywheelController.calculate(m_RightFlywheelEncoder.getVelocity()), -1, 1));
      m_LeftFlywheelMotor.set(MathUtil.clamp(m_LeftFlywheelController.calculate(m_LeftFlywheelEncoder.getVelocity()), -1, 1));

      double turn = m_TurnController.calculate(m_TurnEncoder.getPosition()*360);
      double turnError = m_TurnController.getPositionError();
      double hoodError = turnError - ((currentTurretState.hoodAngle - desiredTurretState.hoodAngle) * (24 / 18));
      m_TurnMotor.set(turn);
      m_HoodMotor.set(m_HoodController.calculate(MathUtil.inputModulus(m_HoodEncoder.getPosition()*360, 0, 360), MathUtil.inputModulus(m_TurnEncoder.getPosition()*360, 0, 360) + hoodError));
    } else {
      m_RightFlywheelMotor.set(0);
      m_LeftFlywheelMotor.set(0);
      m_TurnMotor.set(0);
      m_HoodMotor.set(0);
    }
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
