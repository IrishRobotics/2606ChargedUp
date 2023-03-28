package frc.robot.commands;

import frc.robot.subsystems.ArmSub;

public class LowerArmRetract extends ArmMoveToAngle {

    public LowerArmRetract(ArmSub arm) {
        super(arm, 60, 1.0);
    }

    @Override
    public boolean isFinished() {
        if(armSub.getLimit2().get())
        {
            return true;
        }
        return super.isFinished();
    }
}
