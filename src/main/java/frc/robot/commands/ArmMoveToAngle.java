package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;

public class ArmMoveToAngle extends CommandBase {
    double currentSpeed = 0.0;
    double maxSpeed = .4;
    double maxAccelPerTick = maxSpeed / 25.0;
    double startAngle = 0.0;
    int ticks = 0;
    int ticksToMove = 0;
    int ticksToReachTen = 0;

    protected ArmSub armSub;
    private double endAngle;
    private double scaler;

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addIntegerProperty("ticks", this::getTicks, null); 
      builder.addIntegerProperty("ticksToMove", this::getTicksToMove, null); 
      builder.addIntegerProperty("tickstoReachTen", this::getTicksToReachTen, null);
      builder.addDoubleProperty("currentspeed" , this::getCurrentSpeed, this::setCurrentSpeed);
      builder.addDoubleProperty("maxAccel", this::getMaxAccelPerTick, this::setMaxAccelPerTick);
      

    }

    public void setCurrentSpeed(double currentSpeed) {
        this.currentSpeed = currentSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxAccelPerTick() {
        return maxAccelPerTick;
    }

    public void setMaxAccelPerTick(double maxAccelPerTick) {
        this.maxAccelPerTick = maxAccelPerTick;
    }

    public double getStartAngle() {
        return startAngle;
    }

    public void setStartAngle(double startAngle) {
        this.startAngle = startAngle;
    }

    public int getTicks() {
        return ticks;
    }

    public void setTicks(int ticks) {
        this.ticks = ticks;
    }

    public int getTicksToMove() {
        return ticksToMove;
    }

    public void setTicksToMove(int ticksToMove) {
        this.ticksToMove = ticksToMove;
    }

    public int getTicksToReachTen() {
        return ticksToReachTen;
    }

    public void setTicksToReachTen(int ticksToReachTen) {
        this.ticksToReachTen = ticksToReachTen;
    }

    public ArmSub getArmSub() {
        return armSub;
    }

    public void setArmSub(ArmSub armSub) {
        this.armSub = armSub;
    }

    public double getEndAngle() {
        return endAngle;
    }

    public void setEndAngle(double endAngle) {
        this.endAngle = endAngle;
    }

    public double getScaler() {
        return scaler;
    }

    public void setScaler(double scaler) {
        this.scaler = scaler;
    }

    public ArmMoveToAngle(ArmSub arm, double scaler, double endAngle) {
        this.armSub = arm;
        this.scaler = scaler;
        this.endAngle = endAngle;
        SmartDashboard.putData("ArmMove", this);
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
                currentSpeed = currentSpeed - maxAccelPerTick;
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
        double angle = Math.abs(armSub.updateAngle() - endAngle);
        if (angle < 5) {
            armSub.updateArm(0);
            return true;
        }
        return super.isFinished();
    }

}
