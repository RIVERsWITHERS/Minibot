// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// //import com.studica.frc.AHRS;
// //import com.pathplanner.lib.auto.AutoBuilder;
// //import com.pathplanner.lib.util.ReplanningConfig;
// import com.revrobotics.spark.SparkMax;
// //import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;




// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// //import edu.wpi.first.math.geometry.Pose2d;
// //import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.kinematics.ChassisSpeeds;
// // import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// // import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// // import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
// // import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// //import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;

// public class Drive extends SubsystemBase {
//   private final double defaultSpeedLimit=1;
//   private static final double positionConversionFactor = 14.0/72.0 * Math.PI * Units.inchesToMeters(4) ;  
//   //private static final double volocityConversionFactor = positionConversionFactor / 60 ;


//   private final SparkMax leftMotor = new SparkMax(2, MotorType.kBrushless);
//   private final SparkMax rightMotor = new SparkMax(3, MotorType.kBrushless);
//   public DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
//     // private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
//     // private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
//     // private final DifferentialDriveKinematics  kinematics= new DifferentialDriveKinematics(Units.inchesToMeters(12.25));
//     //  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),0,0 );
//   private final PIDController leftPidController = new PIDController(1, 0, 0);
//   private final PIDController rightPidController = new PIDController(1, 0, 0);
//   private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0,1);
//   private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0,1);
  
//   private double speedLimit;
  
//     /** Creates a new newsubsystem. */
//     public Drive() {
        
//       drive.setDeadband(0.1);
//       drive.setMaxOutput(0.25);
//     //   gyro.zeroYaw();
//     //   leftEncoder.setPositionConversionFactor(positionConversionFactor);
//     //  leftEncoder.setVelocityConversionFactor(volocityConversionFactor);
//     //  rightEncoder.setPositionConversionFactor(positionConversionFactor);
//     //  rightEncoder.setVelocityConversionFactor(volocityConversionFactor);
  
//        drive = new DifferentialDrive(leftMotor, rightMotor);

//         // Enable motor safety to stop motors if commands are delayed
//         drive.setSafetyEnabled(true);

//         // Set the initial speed limit for the motors
//         speedLimit = defaultSpeedLimit;

//         // Display the speed limit on the SmartDashboard for tuning
//         SmartDashboard.putNumber("Speed Limit", speedLimit);
//     }

//     @Override
//     public void periodic() {
//         // Read the speed limit value from the SmartDashboard
//         double newSpeedLimit = SmartDashboard.getNumber("Speed Limit", defaultSpeedLimit);

//         // If the speed limit has changed, update the variable and print a message
//         if (newSpeedLimit != speedLimit) {
//             speedLimit = newSpeedLimit;
//             System.out.println("Speed Limit updated to: " + speedLimit);
//         }
//     }

//     // Implements arcade-style driving using speed (forward/backward) and rotation
//     public void arcadeDrive(double speed, double rotation) {
//         drive.arcadeDrive(speed * speedLimit, rotation * speedLimit);
//     }

//     // Implements tank-style driving using separate speeds for left and right motors
//     public void tankDrive(double leftSpeed, double rightSpeed) {
//         drive.tankDrive(leftSpeed * speedLimit, rightSpeed * speedLimit);
//     }

//     // Stops both motors by setting their output to zero
//     public void stop() {
//         leftMotor.stopMotor();
//         rightMotor.stopMotor();
//     }
// }



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
