package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;

public class ArmMoveToAngle extends CommandBase {
    double currentSpeed = 0.0;
    double maxSpeed = 1.0;
    double maxAccelPerTick = maxSpeed / 25.0;
    double startAngle = 0.0;
    int ticks = 0;
    int ticksToMove = 0;
    int ticksToReachTen = 0;

    protected ArmSub armSub;
    private double endAngle;
    private double scaler;

    public ArmMoveToAngle(ArmSub arm, double scaler, double endAngle) {
        this.armSub = arm;
        this.scaler = scaler;
        this.endAngle = endAngle;
    }

    @Override
    public void execute() {
        super.execute();
        if(ticks==0) {
            currentSpeed = 0.0;
            startAngle = armSub.updateAngle();
        }
        ticks++;
        double angle = armSub.updateAngle();
        if (!isFinished()) {
            if (Math.abs(angle - startAngle) < 10) {
                currentSpeed = currentSpeed + maxAccelPerTick;
            }
            if (Math.abs(angle - endAngle) < 10) {
                currentSpeed = currentSpeed - (maxSpeed / (ticksToReachTen - ticksToMove));
            }
            if (ticksToMove == 0 && (Math.abs(angle - startAngle) > 1)) {
                ticksToMove = ticks;
            }
            if (ticksToReachTen == 0 && (Math.abs(angle - startAngle) > 10)) {
                ticksToReachTen = ticks;
            }
        } else {
            currentSpeed = 0;
        }
        armSub.updateArm(scaler * currentSpeed);
    }

    @Override
    public void initialize() {
        super.initialize();
        ticks = 0;
    }

    @Override
    public boolean isFinished() {
        double angle = Math.abs(armSub.getAngle() - endAngle);
        if (angle < 5) {
            armSub.updateArm(0);
            return true;
        }
        return super.isFinished();
    }

}
