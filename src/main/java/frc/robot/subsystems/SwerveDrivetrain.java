// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem.EncoderID;

public class SwerveDrivetrain extends SubsystemBase {
  /**
   * Inversions account for rotations of the module relative to left or right side of robot.
   * 
   *  CANCoders are setup in Sensors and will have CCW= positve convention. Their offsets
   *  are adjusted by their use in the drive train.  
   */
  boolean kDriveMotorInvert_Right = true;
  boolean kAngleMotorInvert_Right = true;
  boolean kAngleCmdInvert_Right = true;
  boolean kDriveMotorInvert_Left = false;
  boolean kAngleMotorInvert_Left = false;
  boolean kAngleCmdInvert_Left = false;
  /**
   *
   * Modules are in the order of -
   *   Front Left
   *   Front Right
   *   Back Left
   *   Back Right
   * 
   * Positive x values represent moving toward the front of the robot
   * Positive y values represent moving toward the left of the robot
   * All lengths in feet.
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(DriveTrain.XwheelOffset, DriveTrain.YwheelOffset),   // Front Left
    new Translation2d(DriveTrain.XwheelOffset, -DriveTrain.YwheelOffset),  // Front Right
    new Translation2d(-DriveTrain.XwheelOffset, DriveTrain.YwheelOffset),  // Back Left
    new Translation2d(-DriveTrain.XwheelOffset, -DriveTrain.YwheelOffset)  // Back Right
  );

  // sensors and our mk3 modules
  private final Sensors_Subsystem sensors;
  private final Gyro gyro;
  private final SwerveModuleMK3[] modules; 


  public SwerveDrivetrain() {
    sensors = RobotContainer.RC().sensors;
    gyro = sensors;
    
    var MT = CANSparkMax.MotorType.kBrushless;
    modules = new SwerveModuleMK3[] {
      // Front Left
      new SwerveModuleMK3(new CANSparkMax(CAN.DT_FL_DRIVE, MT),  new CANSparkMax(CAN.DT_FL_ANGLE, MT), 
              DriveTrain.CC_FL_OFFSET, sensors.getCANCoder(EncoderID.FrontLeft), 
              kAngleMotorInvert_Left, kAngleCmdInvert_Left, kDriveMotorInvert_Left).setNTPrefix("FL"),
      // Front Right
      new SwerveModuleMK3(new CANSparkMax(CAN.DT_FR_DRIVE, MT),  new CANSparkMax(CAN.DT_FR_ANGLE, MT), 
              DriveTrain.CC_FR_OFFSET, sensors.getCANCoder(EncoderID.FrontRight), 
              kAngleMotorInvert_Right, kAngleCmdInvert_Right, kDriveMotorInvert_Right).setNTPrefix("FR"), 
      // Back Left                    
      new SwerveModuleMK3(new CANSparkMax(CAN.DT_BL_DRIVE, MT),  new CANSparkMax(CAN.DT_BL_ANGLE, MT), 
              DriveTrain.CC_BL_OFFSET,  sensors.getCANCoder(EncoderID.BackLeft), 
              kAngleMotorInvert_Left, kAngleCmdInvert_Left, kDriveMotorInvert_Left ).setNTPrefix("BL"),
      // Back Right
      new SwerveModuleMK3(new CANSparkMax(CAN.DT_BR_DRIVE, MT),  new CANSparkMax(CAN.DT_BR_ANGLE, MT), 
              DriveTrain.CC_BR_OFFSET, sensors.getCANCoder(EncoderID.BackRight),
              kAngleMotorInvert_Right, kAngleCmdInvert_Right, kDriveMotorInvert_Right ).setNTPrefix("BR")
    };
  }

  /**
   * Method to drive the robot using joystick info.
   * 
   *  Length can be meter or ft, just be consistent in field and robot wheel units.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).  [length/s]
   * @param ySpeed Speed of the robot in the y direction (sideways). [length/s]
   * @param rot Angular rate of the robot.  [rad/s]
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // fix speeds if kinematics exceed what the robot can actually do [lenght/s]
    SwerveDriveKinematics.normalizeWheelSpeeds(states, DriveTrain.kMaxSpeed);
    
    // output the angle and velocity for each module
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }


  // used for testing
  public void testDrive(double speed, double angle) { 
    // output the angle and speed (meters per sec)  for each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle))));
    }
  }


  @Override
  public void periodic() {
    //update data from each of the swerve drive modules.
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    //  any sim work for each module
    for (int i = 0; i < modules.length; i++) {
     // modules[i].periodic();
    }
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length -1))  return null;
    return modules[modID];
  }
}
