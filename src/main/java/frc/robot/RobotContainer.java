// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//The Important One
public class RobotContainer {
  

  //Controllers
  private CommandXboxController driveController = new CommandXboxController(Constants.XboxControllerPortDrive);
  private CommandXboxController armController = new CommandXboxController(Constants.XboxControllerPortArm);

  //Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Drive m_Drive = new Drive();
  private final ArmSub m_UpperArm = new ArmSub(Constants.UPPERARM);
  private final ArmSub m_LowerArm = new ArmSub(Constants.LOWERARM);

  //Commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 
  public RobotContainer() {
   
    configureButtonBindings(); //Kinda obvious what this does

    //Drive Command
    m_Drive.setDefaultCommand(new RunCommand(()->{
       m_Drive.updateDrive(-driveController.getLeftY(),-driveController.getLeftX(),driveController.getRightX(),false);
    },m_Drive));


    //Lower Arm Control Command
    m_LowerArm.setDefaultCommand(new RunCommand(()->{
      m_LowerArm.updateArm(armController.getLeftY());
    }, m_LowerArm));


    //Upper Arm Control Command
    m_UpperArm.setDefaultCommand(new RunCommand(()->{
      m_UpperArm.updateArm(armController.getRightY());
    }, m_UpperArm));

  }

  //It does what it says it does :Man_Shrugging:
  private void configureButtonBindings() {
    //Sprint Button
    driveController.b().onTrue(new RunCommand(()->{
      m_Drive.setDriveMode(Constants.driveSpeedKillSprint);
    },m_Drive)).onFalse(new RunCommand(()->{
      m_Drive.setDriveMode(Constants.driveSpeedKillDefault);
    }, m_Drive));

    //Crouch Button
    driveController.x().onTrue(new RunCommand(()->{
      m_Drive.setDriveMode(Constants.driveSpeedKillCrouch);
    },m_Drive)).onFalse(new RunCommand(()->{
      m_Drive.setDriveMode(Constants.driveSpeedKillDefault);
    }, m_Drive));

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { //Prolly need to do this...
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
