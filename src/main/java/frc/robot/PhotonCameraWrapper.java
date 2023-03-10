/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.wpilibj.DriverStation;
 //import frc.robot.Constants.VisionConstants;
 import java.io.IOException;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class PhotonCameraWrapper {
     private PhotonCamera photonCamera;
     private PhotonPoseEstimator photonPoseEstimator;
 
     public PhotonCameraWrapper() {
         // Change the name of your camera here to whatever it is in the PhotonVision UI.
         photonCamera = new PhotonCamera("apriltag");
        // System.out.println("\n\n\n\n\nTHISS\n\n\n\nTHISS\nTHISS\nTHISS\nTHISS\nTHISS\nTHISS\n\n");
 
         try {
             // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
             AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
             // Create pose estimator
             photonPoseEstimator =
                     new PhotonPoseEstimator(
                             fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, Constants.robotToCam);
             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         } catch (IOException e) {
             // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
             // where the tags are.
             DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
             photonPoseEstimator = null;
         }
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
      *     the estimate
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         if (photonPoseEstimator == null) {
             // The field layout failed to load, so we cannot estimate poses.
             return Optional.empty();
         }
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }


     public PhotonTrackedTarget getClosestTarget(){
        PhotonPipelineResult result=photonCamera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget closestTarget = null;
            double closestDist=0;
            for(PhotonTrackedTarget cTarget:result.targets){
                if(closestTarget==null){
                    closestTarget=cTarget;
                    closestDist=Math.abs(cTarget.getBestCameraToTarget().getY());
                }else{
                    double x = Math.abs(cTarget.getBestCameraToTarget().getY());
                    if(x<closestDist){
                        closestDist=x;
                        closestTarget=cTarget;
                    }
                }
            }
            return closestTarget;
        }else{
            return null;
        }
     }
 }