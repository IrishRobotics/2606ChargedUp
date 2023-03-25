package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import frc.robot.subsystems.AdvancedDrive;

public class AutoOutAlign extends CommandBase{
    private PhotonCameraWrapper pcw;
    private AdvancedDrive drive;
    public AutoOutAlign(AdvancedDrive d){
        pcw=new PhotonCameraWrapper();
        drive=d;
    }
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(-Constants.AutoSpeed, 0, 0, false); 
    
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
                return Math.abs(closest.getBestCameraToTarget().getX())<0.5;
            }
        }
    return false;    
}

}
