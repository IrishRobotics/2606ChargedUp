// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ChangeClaw;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.AdvancedDrive;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClawSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmAndDriveBack extends InstantCommand {
  private ArmSub m_Lower;
  private ArmSub m_Upper;
  private ClawSub m_Claw;
  private AdvancedDrive m_drive;
  public ArmAndDriveBack(ArmSub Lower, ArmSub Upper, ClawSub Claw, AdvancedDrive d) {
    m_Lower=Lower;
    m_Upper=Upper;
    m_Claw=Claw;
    m_drive=d;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new MoveArm(m_Upper, .5).withTimeout(4).andThen(new MoveArm(m_Upper, 0).withTimeout(.1)).andThen(new MoveArm(m_Lower, -1).withTimeout(3)).andThen(new MoveArm(m_Lower, 0).withTimeout(0.5)).andThen(new ChangeClaw(m_Claw,true).withTimeout(0.1)).andThen(new MoveArm(m_Lower, 1).withTimeout(4)).andThen(new DriveCommand(m_drive,-.65,0,0).withTimeout(.55)).schedule();
  }
}
