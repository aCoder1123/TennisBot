// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.PID.*;

public class Loader extends SubsystemBase {
    private PIDController m_GateController = new PIDController(kGateP, kGateI, kGateP);
    private PIDController m_LeftLoaderController = new PIDController(kLoaderP, kLoaderI, kLoaderD);
    private PIDController m_RightLoaderController = new PIDController(kLoaderP, kLoaderI, kLoaderD);

    private CANSparkMax m_GateMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax m_LeftLoaderMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax m_RightLoaderMotor = new CANSparkMax(0, MotorType.kBrushless);

    private RelativeEncoder m_GateEncoder;
    private RelativeEncoder m_LeftLoaderEncoder;
    private RelativeEncoder m_RightLoaderEncoder;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final I2C.Port i2cPort2 = I2C.Port.kMXP;
    private final ColorSensorV3 m_TopSensor = new ColorSensorV3(i2cPort);
    private final ColorSensorV3 m_BottomSensor = new ColorSensorV3(i2cPort2);

    private boolean onStandBy = true; 

    public Loader() {
        m_GateEncoder = m_GateMotor.getEncoder();
        m_LeftLoaderEncoder = m_LeftLoaderMotor.getEncoder();
        m_RightLoaderEncoder = m_RightLoaderMotor.getEncoder();
    }

    public boolean isLoaded() {
        if (m_TopSensor.getProximity() > kBallProximity) {return true;}
        return false;
    }

    public boolean onStandBy() {
        return onStandBy;
    }

    public void setStandBy(boolean val) {
        if (!val) {
            onStandBy = false;
        } else if (isLoaded()) {
            onStandBy = true;
        }
    }
    
    public boolean loadBall() {
        if (isLoaded()) {
            if (!onStandBy) {
                while (!(m_LeftLoaderController.atSetpoint() && m_RightLoaderController.atSetpoint())) {
                    m_LeftLoaderMotor
                            .set(m_LeftLoaderController.calculate(m_LeftLoaderEncoder.getVelocity(), kLoaderRPM));
                    m_RightLoaderMotor
                            .set(m_RightLoaderController.calculate(m_RightLoaderEncoder.getVelocity(), kLoaderRPM));
                }
            }
            while (m_BottomSensor.getProximity() < kBallProximity) {
                m_GateMotor.set(m_GateController.calculate(m_GateEncoder.getVelocity(), kGateRPM));
            }
            m_GateMotor.set(0);
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if (onStandBy) {
            m_LeftLoaderMotor.set(m_LeftLoaderController.calculate(m_LeftLoaderEncoder.getVelocity(), kLoaderRPM));
            m_RightLoaderMotor.set(m_RightLoaderController.calculate(m_RightLoaderEncoder.getVelocity(), kLoaderRPM));
        } else {
            m_LeftLoaderMotor.set(0);
            m_RightLoaderMotor.set(0);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
