package frc.robot.subsystems;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCam extends SubsystemBase{
    public PhotonCamera camera;

    public PhotonCam(){
        camera = new PhotonCamera("photonvision");
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    public PhotonPipelineResult getResults(){
        return camera.getLatestResult();
    }

    public void ProcessCurrentScenario(){
        PhotonPipelineResult result=getResults();
        if(result.hasTargets()){
            PhotonTrackedTarget best =result.getBestTarget();
            SmartDashboard.putNumber("CameraPitch", best.getPitch());
            SmartDashboard.putNumber("CameraYaw", best.getYaw());
            SmartDashboard.putNumber("CameraSkew", best.getSkew());
            SmartDashboard.putNumber("CameraArea", best.getArea());
            
            Transform3d pose = best.getBestCameraToTarget();
            SmartDashboard.putNumber("PoseX", pose.getX());
            SmartDashboard.putNumber("PoseY", pose.getY());
            SmartDashboard.putNumber("PoseZ", pose.getZ());


        }
    }
}
