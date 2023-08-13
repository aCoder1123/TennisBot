// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public final class OIConstants {
    public static final int kDriverControllerID = 0;
    public static final int kOperatorControllerID = 1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int kAButtonID = 1;
    public static final int kBButtonID = 2;
    public static final int kXButtonID = 3;
    public static final int kYButtonID = 4;

  }

  public final class MeasurementUtils {
    public static final double kInchesToMeters = 0.0254;
    public static final double kDegreesToRad = Math.PI / 180;
  }

  public final class DrivetrainConstants {
    public static final double kRobotWidthInches = 30;
    public static final double kRobotLengthInches = 30;

    public static final double kMK4ioffsetInches = 2.625;

    public static final double kXModuleOffset = (kRobotLengthInches - kMK4ioffsetInches)
        * MeasurementUtils.kInchesToMeters / 2;
    public static final double kYModuleOffset = (kRobotWidthInches - kMK4ioffsetInches)
        * MeasurementUtils.kInchesToMeters / 2;

    public static final double kWheelDiameterInches = 4;

    public static final double kWheelRadius = (kWheelDiameterInches / 2) * MeasurementUtils.kInchesToMeters;

    public static final int kEncoderResolution = 4096;

    public static final double kSPEED_LIMIT_PORPORTION = 1.0; // ! Adjust to limit the driving speed for events ect.

    public static final double kMaxSpeed = 4.0 * kSPEED_LIMIT_PORPORTION; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

    public static final double kPositionTolerance = 0.01;
  }

  public final class TurretConstants {
    public static final double kTurretGearing = (1 / 5) * (20 / 76);
    public static final double kHoodGearing = (1 / 5) * (20 / 76) * (18 / 24);
  }

  public final class LoaderConstants {

  }

  // TODO add CAN constants and other connections
  public static final class CANConstants {
  }

  // TODO add PID constants
  public static final class PID {
    public static final double kDriveP = 6.0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0.1;

    public static final double kTurnP = 1.5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.1;

    public static final double kModuleDriveP = .20;
    public static final double kModuleDriveI = 0;
    public static final double kModuleDriveD = 0.04;
    public static final double kModuleDriveS = 1;
    public static final double kModuleDriveV = 3;

    public static final double kModuleTurnP = .1;
    public static final double kModuleTurnI = 0;
    public static final double kModuleTurnD = 0.01;
    public static final double kModuleTurnS = 1;
    public static final double kModuleTurnV = .5;

    public static final double kTurretP = 2;
    public static final double kTurretI = 0;
    public static final double kTurretD = 0;

    public static final double kFlywheelP = 3;
    public static final double kFlywheelI = 0.1;
    public static final double kFlywheelD = 0.1;

  }
}
