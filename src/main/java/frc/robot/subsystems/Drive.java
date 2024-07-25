// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {
  private static final double positionConversionFactor = 14.0/72.0 * Math.PI * Units.inchesToMeters(4) ;  
  private static final double volocityConversionFactor = positionConversionFactor / 60 ;


  private final CANSparkMax leftMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final AHRS gyro = new AHRS();
  private final DifferentialDriveKinematics  kinematics= new DifferentialDriveKinematics(Units.inchesToMeters(12.25));
   private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),0,0 );
private final PIDController leftPidController = new PIDController(1, 0, 0);
private final PIDController rightPidController = new PIDController(1, 0, 0);
private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0,1);
private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0,1);


  /** Creates a new newsubsystem. */
  public Drive() {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    drive.setDeadband(0.1);
    gyro.zeroYaw();
    leftEncoder.setPositionConversionFactor(positionConversionFactor);
   leftEncoder.setVelocityConversionFactor(volocityConversionFactor);
   rightEncoder.setPositionConversionFactor(positionConversionFactor);
   rightEncoder.setVelocityConversionFactor(volocityConversionFactor);
  drive.setMaxOutput(0.25);

  
    //AutoBuilder.configureLTV(
     // odometry::getPoseMeters,
     // this::resetPose,
     // this:: getRobotSpeeds,
     // this:: driveWithChassisSpeeds,
     // 0.02,
    //  new ReplanningConfig(),

//()-> false,this




   // );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive Volocity (mps)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Drive Volocity (mps)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Drive Distance (m)", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive Distance (m)", rightEncoder.getPosition());
    odometry.update(gyro.getRotation2d(), getWheelPositions());
  }


public Command getTeleopDriveCommand() {
  return run(()->{ RobotContainer.drive.drive.arcadeDrive(-RobotContainer.controller.getLeftY(), -RobotContainer.controller.getRightX());
 });
}

public DifferentialDriveWheelPositions getWheelPositions(){
  return new DifferentialDriveWheelPositions( 
    leftEncoder.getPosition(), rightEncoder.getPosition());
}
public ChassisSpeeds getRobotSpeeds(){
 return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
 leftEncoder.getVelocity(), rightEncoder.getVelocity()));
}


public void resetPose(Pose2d pose){
odometry.resetPosition(
  gyro.getRotation2d(),
getWheelPositions(),
 pose);

}

public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds){
DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
leftMotor.setVoltage(
  leftPidController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond)
  + leftFF.calculate(wheelSpeeds.leftMetersPerSecond)
);
rightMotor.setVoltage(
  rightPidController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond)
  + rightFF.calculate(wheelSpeeds.rightMetersPerSecond)
);

}







}
