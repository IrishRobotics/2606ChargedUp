// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSub extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonSRX armControl;
  private NetworkTableEntry roll;
  private double angle;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private String netGyro;
  private int Id;
  private double upperLimit;
  private double lowerLimit;
  private double scaler;

  public ArmSub(int ID, String netGyro, double lowerLimit, double upperLimit, int scaler) {
    // Creates Talon Controller Supplied ID from constants through robot container
    armControl = new WPI_TalonSRX(ID);
    armControl.setNeutralMode(NeutralMode.Brake);
    this.netGyro = netGyro;
    initGyro(this.netGyro);
    Id = ID;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.scaler = scaler;
  }

  public ArmSub(int ID, String netGyro) {
    // Creates Talon Controller Supplied ID from constants through robot container
    this(ID, netGyro, 0.0, 0.0, 0);
  }

  @Override
  public void periodic() {
    if (roll == null) {
      this.initGyro(netGyro);
    } else {
      angle = roll.getDouble(0.0) + 180;// This method will be called once per scheduler run
    }
    SmartDashboard.putNumber("angle " + Id, angle);
  }

  protected void initGyro(String name) {
    try {
      if (netGyro != null) {
        NetworkTable datatable = inst.getTable(netGyro);
        roll = datatable.getEntry("roll");
      }
    } catch (Exception e) {

    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getLowerLimit(){return lowerLimit;}
  public double getUpperLimit(){return upperLimit;}

  public void updateArm(double d) { // Command for setting the speed for arms
    SmartDashboard.putNumber("SmartSpeed " + Id, d*scaler);
    if (((d * scaler) > 0) && (angle >= upperLimit)) {
      return;
    }
    if (((d * scaler) < 0) && (angle <= lowerLimit)) {
      return;
    }
    armControl.set(d * Constants.armSpeedKill);
    SmartDashboard.putNumber("MotorSpeed " + Id, d);
  }

  public double getAngle() {
    return angle;
  }
}
