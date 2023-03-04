// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drive extends SubsystemBase  {
   //I want gyro, but Gyro doesnt want me :(
   //Gyro gyro = new AHRS(SerialPort.Port.kMXP);

  private CANSparkMax frontRight = null;
  private CANSparkMax frontLeft = null;
  private CANSparkMax backRight = null;
  private CANSparkMax backLeft = null;

  private MecanumDrive mecanumDrive=null;

  private double speedKill;

  
  public Drive() {
    //Creating our spark motor controllers
    frontLeft=new CANSparkMax(Constants.FLSPARK,MotorType.kBrushless);
    frontRight=new CANSparkMax(Constants.FRSPARK,MotorType.kBrushless);
    backLeft=new CANSparkMax(Constants.BLSPARK,MotorType.kBrushless);
    backRight=new CANSparkMax(Constants.BRSPARK,MotorType.kBrushless);

    //Brake mode... Might change this to be like smoother idk
    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    //ben was here 
    //INVERT AHHHHHHHHHH
    frontRight.setInverted(true);
    backRight.setInverted(true);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);  // Makes our mecanum drive system
   // mecanumDrive.setDeadband(0.15);//controller deadzone // bad // really bad


    speedKill=Constants.driveSpeedKillDefault;
  }

    public double deadzone(double s){
      if(Math.abs(s)<0.1)
        return 0;
      return s;
    }
 

  @Override
  public void periodic() {
   //Not really used... 
  }

  public void updateDrive(double x, double y, double z, boolean cartesian){ //Updates the Drive so we can move
    /* 
    x=0;
    y=0;
    z=0; 
    */
    mecanumDrive.driveCartesian(deadzone(x)*speedKill, -1*deadzone(y)*speedKill, deadzone(z)*speedKill);
  }

  public void setDriveMode(double nsk){ //Sets the different Drive speed Modes (See Button Bindings)
    speedKill=nsk;
  }
}
