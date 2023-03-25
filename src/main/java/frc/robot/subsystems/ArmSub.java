// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private double m_lastUpdate;
  private int dioPort;
  private DigitalInput input;

  public ArmSub(int ID, String netGyro, double lowerLimit, double upperLimit, int scaler, int dioPort) {
    // Creates Talon Controller Supplied ID from constants through robot container
    super();
    armControl = new WPI_TalonSRX(ID);
    armControl.setNeutralMode(NeutralMode.Brake);
    this.netGyro = netGyro;
    initGyro(this.netGyro);
    Id = ID;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.scaler = scaler;
    SmartDashboard.putData("ARM"+ID, this);
    SmartDashboard.putData("ARM MOTOR"+ID,armControl);
    this.dioPort = dioPort;
    if(dioPort!=-1) {
      input = new DigitalInput(dioPort);
      SmartDashboard.putData("LIMIT", input);
    }
    
  }

  public ArmSub(int ID, String netGyro) {
    // Creates Talon Controller Supplied ID from constants through robot container
    this(ID, netGyro, 0.0, 0.0, 0,-1 );
  }

  public int getId() {
    return this.Id;
  }

  @Override
  public void periodic() {
    updateAngle();
  }

  public void updateAngle() {
    if (roll == null) {
      this.initGyro(netGyro);
    } else {
      angle = roll.getDouble(0.0) + 180;// This method will be called once per scheduler run
    }
    SmartDashboard.putNumber("angle " + Id, angle);
  }

  public double getLastUpdate() {
    return m_lastUpdate;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("angle", this::getAngle, null); 
    builder.addDoubleProperty("update", this::getLastUpdate, null);    
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
    
    if (((d * scaler) > 0) && (angle >= upperLimit)) {
      d=0;
    }
    if (((d * scaler) < 0) && (angle <= lowerLimit)) {
      d=0;
    }
    if(dioPort!=-1) {
      if (input.get() && d<0 ) {
        d=0;
      }
    }
    double update = d * Constants.armSpeedKill;
    m_lastUpdate=d;
    armControl.set(update);
    SmartDashboard.putNumber("MotorSpeed " + Id, update);
  }

  public double getAngle() {
    return angle;
  }
}
