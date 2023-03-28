// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import frc.robot.subsystems.AdvancedDrive;
import frc.robot.subsystems.ExampleSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DRIVEBACK extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final AdvancedDrive m_Drive;
  private PhotonCameraWrapper pcw;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DRIVEBACK(AdvancedDrive subsystem) {
    m_Drive = subsystem;
    pcw=new PhotonCameraWrapper();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.drive(0, -Constants.AutoSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget closest = pcw.getClosestTarget();
    if(closest!=null){
        if(closest.getFiducialId()==Constants.AutoFeducial){
            return Math.abs(closest.getBestCameraToTarget().getY())>0.5;
        }
    }
    return false;  
  }
}
