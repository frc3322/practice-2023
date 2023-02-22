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
    public static final double kTurnP = 0.03;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.0075;
    public static final double kTurnToleranceDeg = 0.05;
    public static final double kTurnRateToleranceDegPerS = 10;

    public static final double kDriveP = 0.03; //0.03
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;//.0075;
    public static final double kDriveToleranceDeg = 0.05;
    public static final double kDriveRateToleranceMetersPerS = .01;

    public static final double encoderTicsPerFoot = 6.84;
  }

  public static final class CAN {
    // Drivetrain motors
    public static final int FL = 2;
    public static final int FR = 3;
    public static final int BL = 4;
    public static final int BR = 11;
    //test Motor
    public static final int testMotor = 8;
  }
  public static final class PWM {
    public static final int S1 = 8;
    public static final int S2 = 9;
  }

  public static final class SysID{
    public static final double trackwidth = .9372;
    //public static final double ks = 0.31008;
    //public static final double ks = 0.1474;
    public static final double ks = 0.17373;
    //public static final double kv = 4.9366;
    //public static final double kv = 2.4;
    public static final double kv = 3.3764 * .5 * trackwidth;
    //public static final double ka = 7.5013;
    //public static final double ka = 0.68337;
    public static final double ka = 0.48994 * .5 * trackwidth;

    //public static final double kp = 0.00018;
    public static final double kp = 0.00052787;

    public static final double MaxSpeed = 0.1;
    public static final double MaxAcceleration = 0.1;

   
    public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(trackwidth);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class I2CConst{
    public static final int COMMAND_REGISTER_BIT = 0x80;
    public static final int MULTI_BYTE_BIT= 0x20;

    //clear byte address
    public static final int CDATA_REGISTER = 0x14;
    //red byte address
    public static final int RDATA_REGISTER = 0x16;
    //green byte address
    public static final int GDATA_REGISTER = 0x18;
    //blue byte address
    public static final int BDATA_REGISTER = 0x1A;
    //proximity byte adress
    public static final int PDATA_REGISTER = 0x1C;
  }
}
