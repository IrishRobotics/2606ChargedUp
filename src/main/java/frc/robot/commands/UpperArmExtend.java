package frc.robot.commands;

import frc.robot.subsystems.ArmSub;

public class UpperArmExtend extends ArmMoveToAngle {

    public UpperArmExtend(ArmSub arm) {
        super(arm, 1, 10);
    }
    
}
