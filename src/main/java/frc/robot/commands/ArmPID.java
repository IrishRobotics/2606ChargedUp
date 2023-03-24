package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Drive;

/** A command that will turn the robot to the specified angle. */
public class ArmPID extends PIDCommand {
  boolean valid;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public ArmPID(double targetAngleDegrees, ArmSub arm, double outputScaler) {
    super(
        new PIDController(Constants.kPArm, Constants.kIArm, Constants.kDArm),
        // Close loop on heading
        arm::getAngle,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> arm.updateArm(outputScaler*output),
        // Require the drive
        arm);

    if(targetAngleDegrees>arm.getLowerLimit()&&targetAngleDegrees<arm.getUpperLimit()){
      valid=true;
    }else{
      valid=false;
    }
    

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.kPosTolArm, Constants.kVelTolArm);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    // if(!valid){
    //   return true;
    // }
    return getController().atSetpoint();
  }
}