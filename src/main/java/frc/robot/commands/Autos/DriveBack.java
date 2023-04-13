// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.AdvancedDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBack extends CommandBase {
  AdvancedDrive m_Drive;
  public DriveBack( AdvancedDrive Drive) {
    // Use addRequirements() here to declare subsystem dependencies.
   this.m_Drive=Drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //new m_Drive.drive(0, -1, 0, false).withTimeOut();
   
  //  new RunCommand(() -> {
  //   m_Drive.drive(0, -1, 0, false);
  // }, m_Drive).withTimeout(1);
  new DriveCommand(m_Drive,-.65,0,0).withTimeout(.35).schedule();
  }
  // public void execute(){
  //   new DriveCommand(m_Drive,0,-1,0).withTimeout(1);
  // }
}
