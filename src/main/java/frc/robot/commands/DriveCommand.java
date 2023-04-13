// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AdvancedDrive;

public class DriveCommand extends CommandBase {
  /** Creates a new Drive. */
  AdvancedDrive m_Drive;
  double mx;
  double my;
  double mz;
  public DriveCommand(AdvancedDrive d,double x, double y, double z) {
    // Use addRequirements() here to declare subsystem dependencies.
    mx=x;
    my=y;
    mz=z;
    m_Drive=d;
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("\n\n\n\n\n\n\\nn\n DOING SOMETHING");
    m_Drive.drive(mx, my, mz, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
