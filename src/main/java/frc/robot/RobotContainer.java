// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmPID;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AdvancedDrive;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClawSub;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoOutAlign;
import frc.robot.commands.DRIVEBACK;

//The Important One
public class RobotContainer {

  // Controllers
  private CommandXboxController driveController = new CommandXboxController(Constants.XboxControllerPortDrive);
  private CommandXboxController armController = new CommandXboxController(Constants.XboxControllerPortArm);

  // Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // public final Drive m_Drive = new Drive();
  private final ArmSub m_UpperArm = new ArmSub(Constants.UPPERARM, "E468A8969F1A5621");
  private final ArmSub m_LowerArm = new ArmSub(Constants.LOWERARM, "E460BD10C32C3326", 0, 8,7); 
  private final ClawSub m_Claw = new ClawSub(Constants.ClawChannel);
  private final AdvancedDrive m_Drive2 = new AdvancedDrive();

  // Commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoOutAlign m_AutoOutAlignComm = new AutoOutAlign(m_Drive2);
  private final DRIVEBACK m_Driveback = new DRIVEBACK(m_Drive2);
  private ArmPID p1;
  private ArmPID p2;

  public RobotContainer() {

    configureButtonBindings(); // Kinda obvious what this does
    PowerDistribution powerD = new PowerDistribution();
    SmartDashboard.putData("PDH", powerD);
    // Drive Command -- basic
    // m_Drive.setDefaultCommand(new RunCommand(() -> {
    // m_Drive.updateDrive(-driveController.getLeftY(), -driveController.getLeftX(),
    // driveController.getRightX(), false);
    // }, m_Drive));

    // Drive Command -- advanced
    m_Drive2.setDefaultCommand(new RunCommand(() -> {
      m_Drive2.drive(-driveController.getLeftY(), driveController.getLeftX(), driveController.getRightX(), false);
    }, m_Drive2));

    // Lower Arm Control Command
    m_LowerArm.setDefaultCommand(new RunCommand(() -> {
      m_LowerArm.updateArm(armController.getLeftY());
    }, m_LowerArm));//+Y is in -- meaning smaller angle

    // Upper Arm Control Command
    m_UpperArm.setDefaultCommand(new RunCommand(() -> {
      m_UpperArm.updateArm(-armController.getRightY());
    }, m_UpperArm));

  }

  // It does what it says it does :Man_Shrugging:
  private void configureButtonBindings() {
    // Sprint Button
    // driveController.b().onTrue(new RunCommand(() -> {
    //   m_Drive2.setDriveMode(Constants.driveSpeedKillSprint);
    // }, m_Drive2));

    // // Crouch Button
    // driveController.a().onTrue(new RunCommand(() -> {
    //   m_Drive2.setDriveMode(Constants.driveSpeedKillCrouch);
    // }, m_Drive2));

    // // Normal speed button
    // driveController.x().onTrue(new RunCommand(() -> {
    //   m_Drive2.setDriveMode(Constants.driveSpeedKillDefault);
    // }, m_Drive2));

    p1 = new ArmPID(Constants.lowerArmDriveAng, m_LowerArm, 1);
    p2 = new ArmPID(Constants.upperArmPickUpAng, m_UpperArm, -1.0);
    SmartDashboard.putData("Lower Arm Test", p1);
    SmartDashboard.putData("Upper Arm Test", p2);

    SmartDashboard.putData("Auto", m_AutoOutAlignComm.andThen(m_Driveback));

    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(320, 240);

    // lowerArm outputScaler needs to be -1 i think some inverse relation
    // upperArm outputScaler needs to be -1 upsidedown for angle
    /*armController.b().onTrue(new ArmPID(Constants.lowerArmDriveAng, m_LowerArm, -1.0)
        .andThen(new ArmPID(Constants.upperArmDriveAng, m_UpperArm, 1.0)));

    armController.a().onTrue(new ArmPID(Constants.lowerArmPickUpAng, m_LowerArm, -1.0)
        .andThen(new ArmPID(Constants.upperArmPickUpAng, m_UpperArm, -1.0)));

    armController.x().onTrue(new ArmPID(Constants.upperArmFullExtendAng, m_UpperArm, 1.0)
        .andThen(new ArmPID(Constants.lowerArmFullExtendAng, m_LowerArm , -1.0)));
*/
    armController.rightBumper().onTrue(new RunCommand(() -> {
      m_Claw.setSolenoid(true);
    }, m_Claw));
    armController.leftBumper().onTrue(new RunCommand(() -> {
      m_Claw.setSolenoid(false);
    }, m_Claw));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { // Prolly need to do this...
    // An ExampleCommand will run in autonomous
    return m_AutoOutAlignComm.andThen(m_Driveback);
  }
  public void passSpeed(double speedToPass){
    SmartDashboard.putNumber("RobotSpeed", speedToPass);
    m_Drive2.setDriveMode(speedToPass);
  }
}
