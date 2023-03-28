package frc.robot.commands;

import frc.robot.subsystems.ArmSub;

public class UpperArmExtend extends ArmMoveToAngle {

    public UpperArmExtend(ArmSub arm, double scaler, double endAngle) {
        super(arm, -1, 10);
    }
    
}
