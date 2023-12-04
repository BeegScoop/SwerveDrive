// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    //MotorConversionFactors
    public static final double kDriveEncoderRot2Meter = 0;
    public static final double kDriveEncoderRPM2MeterPerSec = 0;
    public static final double kTurnEncoderRot2Rad = 0;
    public static final double kTurnEncoderRPM2RadPerSec = 0;
  }
  public static class ModuleConstants{
    //confirm these values -  confirmed
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/6.75;
    public static final double kTurningMotorGearRatio = 1/(150/7);
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60;
    //wtf this do
    public static final double kPTurning = 0.5;
  }
  public static class DriveConstants{
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(26);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
      );



    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    
    
    //still need to be confirmed
    public static final int kFrontLeftDriveMotorPort = 13;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 8;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 10;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 23;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    //NEED TO FIGURE OUT OFFSET USING READINGS
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    //CHECK CONTROLLER VALUES
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.1;
}
}
