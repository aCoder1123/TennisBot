// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.PID.*;



public class Turret extends SubsystemBase {
  private PIDController m_RightFlywheelController = new PIDController(kFlywheelP, kFlywheelI, kFlywheelP);
  private PIDController m_LeftFlywheelController = new PIDController(kFlywheelP, kFlywheelI, kFlywheelP);
  private PIDController m_TurnController = new PIDController(kTurretP, kTurretI, kTurretD);
  private PIDController m_HoodController = new PIDController(kTurretP, kTurretI, kTurretD);

  private SimpleMotorFeedforward m_RightFlywheelFeedForward = new SimpleMotorFeedforward(kFlywheelS, kflywheelV);
  private SimpleMotorFeedforward m_LeftFlywheelFeedForward = new SimpleMotorFeedforward(kFlywheelS, kflywheelV);

  private CANSparkMax m_LeftFlywheelMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_RightFlywheelMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_TurnMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_HoodMotor = new CANSparkMax(0, MotorType.kBrushless);

  private RelativeEncoder m_RightFlywheelEncoder;
  private RelativeEncoder m_LeftFlywheelEncoder;
  private RelativeEncoder m_TurnEncoder;
  private RelativeEncoder m_HoodEncoder;

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // TODO ids
  // TODO readout

  //! TUNE PID AND FF

  class TurretState {
    public double rpm = 0;
    public double turretRot = 0;
    public double hoodAngle = 0;
  }

  TurretState currentTurretState = new TurretState();
  TurretState desiredTurretState = new TurretState();

  private boolean onStandBy = true;
  private boolean search = true;

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
  // todo empirically determine formula
  public double velocityToRPM(double velocity) {
    double rpm = (1 * Math.pow(velocity, 2) + 1 * velocity + .1);
    return MathUtil.clamp(rpm, -1, 1);
  }

  public TurretState getTurretPos() {
    return currentTurretState;
  }

  public void setDesiredState(TurretState desiredPos) {
    desiredTurretState = desiredPos;
    desiredTurretState.hoodAngle = MathUtil.clamp(desiredPos.hoodAngle, 22.5, 45);
    m_TurnController.setSetpoint(desiredPos.turretRot);
    // m_HoodController.setSetpoint(desiredPos.turretRot -
    // ((currentTurretState.hoodAngle - desiredPos.hoodAngle) * (24 / 18)));
    m_LeftFlywheelController.setSetpoint(desiredTurretState.rpm);
    m_RightFlywheelController.setSetpoint(desiredTurretState.rpm);
    
  }

  public boolean getTV() {
    if (limelightTable.getEntry("tv").getInteger(0) == 1) {
      return true;
    }
    return false;
  }

  public double getTX() {
    return limelightTable.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return limelightTable.getEntry("ty").getDouble(0);
  }

  public double getTA() {
    return limelightTable.getEntry("ta").getDouble(0);
  }

  public double getTS() {
    return limelightTable.getEntry("ts").getDouble(0);
  }
  
  // TODO - MATH
  public TurretState getShotData(Pose2d robotPos) {
    TurretState shotData = new TurretState();
    return shotData;
  }

  public void prepareShot(Pose2d robotPose2d) {
    desiredTurretState = getShotData(robotPose2d);
    onStandBy = true;
  }

  public boolean shotReady() {
    if (m_HoodController.atSetpoint() && m_TurnController.atSetpoint() && m_RightFlywheelController.atSetpoint()
        && m_LeftFlywheelController.atSetpoint() && getTV()) {
      currentTurretState = desiredTurretState;
      return true;
    }
    return false;
  }

  public boolean getSearch() {
    return this.search;
  }

  public void setSearch(boolean mode) {
    this.search = mode;
  }

  public boolean getStandby() {
    return this.onStandBy;
  }

  public void setStandBy(boolean val) {
    this.onStandBy = val;
  }

  public double getRPM() {
    return this.desiredTurretState.rpm;
  }

  public double getRotation() {
    return this.desiredTurretState.turretRot;
  }

  public double getAngle() {
    return this.desiredTurretState.hoodAngle;
  }

  public void startReadout() {
    SendableBuilderImpl m_builder = new SendableBuilderImpl();
    m_builder.setSmartDashboardType("Turret");
    m_builder.addBooleanProperty("Standby", this::getStandby, this::setStandBy);
    m_builder.addBooleanProperty("Search", this::getSearch, this::setSearch);
    m_builder.addDoubleProperty("RPM", this::getRPM, null);
    m_builder.addDoubleProperty("Rotation", this::getRotation, null);
    m_builder.addDoubleProperty("Angle", this::getAngle, null);

    m_builder.close();
  }


  //TODO implement search
  @Override
  public void periodic() {
    
    if (onStandBy) {
      double rightFlywheel = m_RightFlywheelController.calculate(m_RightFlywheelEncoder.getVelocity()) + m_RightFlywheelFeedForward.calculate(m_RightFlywheelEncoder.getVelocity());
      double leftFlywheel = m_LeftFlywheelController.calculate(m_LeftFlywheelEncoder.getVelocity()) + m_LeftFlywheelFeedForward.calculate(m_LeftFlywheelEncoder.getVelocity());
      m_RightFlywheelMotor
          .set(MathUtil.clamp(rightFlywheel, -1, 1));
      m_LeftFlywheelMotor
          .set(MathUtil.clamp(leftFlywheel, -1, 1));

      double turn = m_TurnController.calculate(m_TurnEncoder.getPosition() * 360);
      double turnError = m_TurnController.getPositionError();
      double hoodError = turnError - ((currentTurretState.hoodAngle - desiredTurretState.hoodAngle) * (24 / 18));
      m_TurnMotor.set(turn);
      m_HoodMotor.set(m_HoodController.calculate(MathUtil.inputModulus(m_HoodEncoder.getPosition() * 360, 0, 360),
          MathUtil.inputModulus(m_TurnEncoder.getPosition() * 360, 0, 360) + hoodError));
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
