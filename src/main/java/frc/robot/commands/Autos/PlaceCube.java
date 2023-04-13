// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmMoveToAngle;
import frc.robot.commands.LowerArmExtend;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClawSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCube extends InstantCommand {
  private ArmSub m_Upper;
  private ArmSub m_Lower;
  private ClawSub m_Claw;

  public PlaceCube(ArmSub Upper, ArmSub Lower, ClawSub claw) {
    m_Upper=Upper;
    m_Lower=Lower;
    m_Claw=claw;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new ArmMoveToAngle(m_Upper, 1, 345).andThen(new ArmMoveToAngle(m_Lower, -1, 90)).andThen(new ArmMoveToAngle(m_Upper, 1, 345)).andThen(new LowerArmExtend(m_Lower)).andThen(()->{m_Claw.setSolenoid(true);}, m_Claw).andThen(new ArmMoveToAngle(m_Lower,1,Constants.lowerArmDriveAng)).schedule();
  }
}
