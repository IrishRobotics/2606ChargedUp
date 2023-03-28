package frc.robot.commands;

import frc.robot.subsystems.ArmSub;

public class LowerArmExtend extends ArmMoveToAngle {

    public LowerArmExtend(ArmSub arm) {
        super(arm, 135, 1.0);
    }

    @Override
    public boolean isFinished() {
        if(armSub.getLimit1().get())
        {
            return true;
        }
        return super.isFinished();
    }

    
}
