// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSub extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonSRX armControl;

  public ArmSub(int ID) {
    // Creates Talon Controller Supplied ID from constants through robot container
    armControl = new WPI_TalonSRX(ID);
    armControl.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void updateArm(double d) { // Command for setting the speed for arms

    armControl.set(d * Constants.armSpeedKill);
  }

  public double getAngle() {
    return 0.0;
  }
}
