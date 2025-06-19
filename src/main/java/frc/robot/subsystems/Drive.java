
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  // Constants
  private static final double defaultSpeedLimit = 1.0;
  private static final double positionConversionFactor = (14.0 / 72.0) * Math.PI * Units.inchesToMeters(4);
  
  // Motor Controllers
  private final SparkMax leftMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(3, MotorType.kBrushless);

  // Drive System
  private final DifferentialDrive drive;

  // PID and Feedforward
  private final PIDController leftPidController = new PIDController(1, 0, 0);
  private final PIDController rightPidController = new PIDController(1, 0, 0);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 1);
  private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0, 1);

  // Speed limit (tunable)
  private double speedLimit;

  /** Creates a new Drive subsystem. */
  public Drive() {
    // Configure left motor
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted = true;
    leftMotor.setConfig(leftConfig);

    // Configure right motor
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted = false;
    rightMotor.setConfig(rightConfig);

    // Initialize drive
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(0.1);
    drive.setMaxOutput(0.25);
    drive.setSafetyEnabled(true);

    // Initialize speed limit
    speedLimit = defaultSpeedLimit;
    SmartDashboard.putNumber("Speed Limit", speedLimit);
  }

  @Override
  public void periodic() {
    // Check for updated speed limit from SmartDashboard
    double newSpeedLimit = SmartDashboard.getNumber("Speed Limit", defaultSpeedLimit);
    if (newSpeedLimit != speedLimit) {
      speedLimit = newSpeedLimit;
      System.out.println("Speed Limit updated to: " + speedLimit);
    }
  }

  /** Arcade drive method for single-stick control */
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed * speedLimit, rotation * speedLimit);
  }

  /** Tank drive method for dual-stick control */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed * speedLimit, rightSpeed * speedLimit);
  }

  /** Stop all motors immediately */
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
