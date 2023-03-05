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
    // CAN ID's for the drive wheels
    public static final int FRSPARK = 13;
    public static final int BRSPARK = 14;
    public static final int BLSPARK = 15;
    public static final int FLSPARK = 12;

    // Controller ports
    public static final int XboxControllerPortDrive = 0;
    public static final int XboxControllerPortArm = 1;

    // Variable Drive Speeds
    public static final double driveSpeedKillDefault = .5;
    public static final double driveSpeedKillCrouch = .1;
    public static final double driveSpeedKillSprint = 1;

    // CAN ID's for Arm Segments
    public static final int UPPERARM = 3;
    public static final int LOWERARM = 1;

    // Speed of Arm
    public static final double armSpeedKill = .35;

    // Arm Angle Pre-sets
    public static final double lowerArmLowGoalAng = 0.0;
    public static final double upperArmLowGoalAng = 0.0;

    // Arm PID Constants
    public static final double kPArm = 0.0;
    public static final double kIArm = 0.0;
    public static final double kDArm = 0.0;

    public static final double kPosTolArm = 0.0;
    public static final double kVelTolArm = 0.0;

    // Claw Solenoid Port
    public static final int ClawChannel = 0;

    // goinkers
    // ploinkers
    // doinkers
    // woinkers
    // poilnt
    // badoinkers
    // mhhmhhhhmmmmmmmmmhhhh
    // mmmmmmmmhhhhhhh

}
