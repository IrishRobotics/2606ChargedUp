// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class AdvancedDrive extends SubsystemBase {
  // drive motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax (Constants.FLSPARK, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax   (Constants.BLSPARK, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax (Constants.FRSPARK, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax  (Constants.BRSPARK, MotorType.kBrushless);

  // Drive's encoders
  private final RelativeEncoder m_frontLeftEncoder = frontLeftMotor.getEncoder();
  private final RelativeEncoder m_rearLeftEncoder = rearLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = frontRightMotor.getEncoder();
  private final RelativeEncoder m_rearRightEncoder = rearRightMotor.getEncoder();

  private double speedMultiplier;

  // the robot's drive
  private final MecanumDrive m_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  // Gyro
  private final AHRS m_navX2 = new AHRS(SPI.Port.kMXP);

  private final MecanumDrivePoseEstimator m_poseEstimator;

  public PhotonCameraWrapper pcw;
  private final Field2d m_fieldSim;
  // private final DifferentialDrivetrainSim m_drivetrainSimulator =
  //           new DifferentialDrivetrainSim(
  //                   m_drivetrainSystem,
  //                   DCMotor.getCIM(2),
  //                   8,
  //                   DriveTrainConstants.kTrackWidth,
  //                   DriveTrainConstants.kWheelRadius,
  //                   null); ---------------------------NO MACANUM EQUIVALANT OF DIFFERENTIALDRIVESIM

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          Constants.mecanumKinie,
          m_navX2.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new ExampleSubsystem. */
  public AdvancedDrive() {
    m_navX2.calibrate();
    speedMultiplier=Constants.driveSpeedKillDefault;
    pcw=new PhotonCameraWrapper();
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
    // set invert the right side
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // set deadband
    m_drive.setDeadband(Constants.kDeadband);

    // set MaxOutput for drive's motors
    m_drive.setMaxOutput(Constants.kMaxOutput);

    // Sets the distance per pulse for the encoders
    // so we don't have to multiply the conversion factor each time we get the encoder's values
    // Note that the distance is in meter because odometer use meter unit
    m_frontLeftEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_rearLeftEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_frontRightEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);
    m_rearRightEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulseMeters);

    resetEncoders();
    resetGyro();

    m_poseEstimator =
  new MecanumDrivePoseEstimator(
          Constants.mecanumKinie, m_navX2.getRotation2d(), getCurrentWheelDistances(), new Pose2d());
  }

  @Override
  public void periodic() {
   // m_odometry.update(m_navX2.getRotation2d(), getCurrentWheelDistances());
    updateOdometry();


  }

  public void updateOdometry() {
    m_poseEstimator.update(
            m_navX2.getRotation2d(),getCurrentWheelDistances());

    Optional<EstimatedRobotPose> result =
            pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        m_poseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
        // move it way off the screen to make it disappear
        m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

   // m_fieldSim.getObject("Actual Pos").setPose(m_drivetrainSimulator.getPose());
    m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
  
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Below are functions for Drive subsystem ====================================  
  // use 3 parameters for Robot-concentric control
  public void drive (double xSpeed, double ySpeed, double rot){
    m_drive.driveCartesian(xSpeed*speedMultiplier, ySpeed *speedMultiplier, rot*speedMultiplier);
  }

  // use 4 parameters for field-centric control
  public void drive(double xSpeed, double ySpeed, double rot, boolean useGyro) {
   // drive(0,0,0);
    
     if(useGyro)
      m_drive.driveCartesian(deadzone(xSpeed)*speedMultiplier, deadzone(ySpeed) *speedMultiplier, deadzone(rot)*speedMultiplier, m_navX2.getRotation2d());
     else
       m_drive.driveCartesian(deadzone(xSpeed)*speedMultiplier, deadzone(ySpeed) *speedMultiplier, deadzone(rot)*speedMultiplier);
  }
  public double deadzone(double s){
    if(Math.abs(s)<0.1)
      return 0;
    return s;
  }
  public MecanumDrive getMecanumDrive(){
    return m_drive;
  }

  // set motors on Brake or Coast mode
  public void enableMotors(boolean on) {
    CANSparkMax.IdleMode mode;

    if (on) mode = CANSparkMax.IdleMode.kBrake;
    else mode = CANSparkMax.IdleMode.kCoast;

    frontLeftMotor.setIdleMode(mode);
    rearLeftMotor.setIdleMode(mode);
    frontRightMotor.setIdleMode(mode);
    rearRightMotor.setIdleMode(mode);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

/** Sets the drive MotorController to a voltage. */
public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
  frontLeftMotor.setVoltage(volts.frontLeftVoltage);
  rearLeftMotor.setVoltage(volts.rearLeftVoltage);
  frontRightMotor.setVoltage(volts.frontRightVoltage);
  rearRightMotor.setVoltage(volts.rearRightVoltage);

  m_drive.feed();
}

  // Below are functions for encoders =====================================  
  public void resetEncoders(){
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  public double getFLEncoderFt(){
    return Units.metersToFeet(m_frontLeftEncoder.getPosition());
  }

  public double getRLEncoderFt(){
    return Units.metersToFeet(m_rearLeftEncoder.getPosition());
  }

  public double getFREncoderFt(){
    return Units.metersToFeet(m_frontRightEncoder.getPosition());
  }

  public double getRREncoderFt(){
    return Units.metersToFeet(m_rearRightEncoder.getPosition());
  }

  public double getAverageEncoderDistance() {
    return (m_frontLeftEncoder.getPosition() + m_frontRightEncoder.getPosition()) / 2.0;
  }

  // Gets the current wheel distance measurements and put them in a MecanumDriveWheelPositions object
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
  }

  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  // below are functions for Gyro =================================

  public void resetGyro(){
    m_navX2.reset();
  }

  public AHRS getGyro(){
    return m_navX2;
  }

  public double getHeading() {
    //return Math.IEEEremainder(m_navX2.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_navX2.getRotation2d().getDegrees();

  }

  public float getGyroPitch(){
    return getGyro().getPitch(); 
  }

  public float getGyroRoll(){
    return getGyro().getRoll();
  }

  // The turn rate of the robot, in degrees per second
  public double getTurnRate() {
    return m_navX2.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  // Below are functions for Odometry ==========================================================================

  // get the current estimated pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(m_navX2.getRotation2d(), getCurrentWheelDistances(), pose);
  }

public void setDriveMode(double drivespeedkillsprint) {

}

}

