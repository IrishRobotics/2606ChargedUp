// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

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
    // CAN ID's for the drive wheels
    public static final int FRSPARK = 13;
    public static final int BRSPARK = 14;
    public static final int BLSPARK = 15;
    public static final int FLSPARK = 12;

    // Drive kinie poo
    /*
     * front Left X/Y: .41,.25
     * front Right X/Y: .12,.25
     * back Left X/Y: .41,.28
     * back Right x/Y:.12,.28
     */
    public static final MecanumDriveKinematics mecanumKinie = new MecanumDriveKinematics(new Translation2d(.42,.25),new Translation2d(.12,.25),new Translation2d(.41,.28),new Translation2d(.12,.28));

    // Controller ports
    public static final int XboxControllerPortDrive = 0;
    public static final int XboxControllerPortArm = 1;

    // DeadBand && outputs
    public static double kDeadband = 0.1;
    public static double kMaxOutput = 4;
    public static boolean kGyroReversed = false;

    // Encoder stuff
    public static double wheelDiameter = 0.2032;
    public static double gearRatio=10.71;
    public static double countsPerRev = 42;
    public static final double kEncoderDistancePerPulseMeters =
    (wheelDiameter * Math.PI) / countsPerRev * gearRatio;


    // Variable Drive Speeds
    public static final double driveSpeedKillDefault = .25;
    public static final double driveSpeedKillCrouch = .2;
    public static final double driveSpeedKillSprint = 1;
    public static final double strafeSpeedKill=.75;

    // CAN ID's for Arm Segments
    public static final int UPPERARM = 3;
    public static final int LOWERARM = 1;

    // Speed of Arm
    public static final double armSpeedKill = .37;

    // Arm Angle Pre-sets
    public static final double lowerArmPickUpAng = 90.0;
    public static final double upperArmPickUpAng = 45.0;

    public static final double lowerArmDriveAng = 40.0;
    public static final double upperArmDriveAng = 60.0;

    public static final double lowerArmFullExtendAng = 90.0;
    public static final double upperArmFullExtendAng = 5.0;

    public static final double lowerArmLowGoalAng = 0.0;
    public static final double upperArmLowGoalAng = 0.0;

    public static final double lowerArmMidGoalAng = 0.0;
    public static final double upperArmMidGoalAng = 0.0;

    public static final double lowerArmHighGoalAng = 0.0;
    public static final double upperArmHighGoalAng = 0.0;

    // Arm PID Constants
    public static final double kPArm = 0.02;
    public static final double kIArm = 0.0;
    public static final double kDArm = 0.04;

    public static final double kPosTolArm = 2.5;
    public static final double kVelTolArm = 4.0;

    // Claw Solenoid Port
    public static final int ClawChannel = 1;
    
    //Camera stuff
    public static Transform3d robotToCam = new Transform3d(new Translation3d(.4,.08,.25),new Rotation3d());

    //Auto Stuff
    public static int AutoFeducial = 6;
    public static double AutoSpeed = 0.25;
    

    // goinkers
    // ploinkers
    // doinkers
    // woinkers
    // poilnt
    // badoinkers
    // mhhmhhhhmmmmmmmmmhhhh
    // mmmmmmmmhhhhhhh

}
