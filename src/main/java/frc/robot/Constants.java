// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
  }

  public static final class DriveConstants {
    public static final double kTurnP = 0.001;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10;
  }

  public static final class CAN {
    // Drivetrain motors
    public static final int FL = 2;
    public static final int FR = 3;
    public static final int BL = 4;
    public static final int BR = 11;
  }

  public static final class SysID{
    public static final double ks = -0.31008;
    public static final double kv = 4.9366;
    public static final double ka = 7.5013;
    public static final double kp = 0.018;

    public static final double MaxSpeed = 3;
    public static final double MaxAcceleration = 1;

    public static final double trackwidth = .9372;
    public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(trackwidth);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    

  }
}
