// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Sensors_Subsystem.EncoderID;

public class SwerveDrivetrain extends SubsystemBase {

  
  /**
   * Inversions 
   * 
   *  CANCoders are setup in Sensors and will have CCW= positve convention. Their offsets
   *  are adjusted by their use in the drivetraind.  
   */
  boolean kDriveMotorInvert_Right = true;
  boolean kAngleMotorInvert_Right = true;
  boolean kAngleCmdInvert_Right = true;
  boolean kDriveMotorInvert_Left = false;
  boolean kAngleMotorInvert_Left = false;
  boolean kAngleCmdInvert_Left = false;


  /***
   * 
   * 4/14/21   RHS is spliting encoders,  RB_int =-48  RB_CC = +52  actual = 45 CCW(pos)
   *                                      RF_int =-38  RF_CC = +49  actual = 45 CCW(pos)
   * 
   *  Test performed: align to zero, calibrate with offset, then manually move wheel to 45 deg.
   * 
   * 
   * TODD:  fix the rotation sign between CC and internal.
   * 
   */
  /**
   *
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(DriveTrain.XwheelOffset),   // Front Left
      Units.inchesToMeters(DriveTrain.YwheelOffset)
    ),
    new Translation2d(
      Units.inchesToMeters(DriveTrain.XwheelOffset),   // Front Right
      Units.inchesToMeters(-DriveTrain.YwheelOffset)
    ),
    new Translation2d(
      Units.inchesToMeters(-DriveTrain.XwheelOffset),  // Back Left
      Units.inchesToMeters(DriveTrain.YwheelOffset)
    ),
    new Translation2d(
      Units.inchesToMeters(-DriveTrain.XwheelOffset),  // Back Right
      Units.inchesToMeters(-DriveTrain.YwheelOffset)
    )
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

  public double angleFix(double angle) {
    if (angle > 180){
      return angle-360;
    } else {
    return angle;
    }
  }

  @Override
  public void periodic() {
    //update internal angle 
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
    // This method will be called once per scheduler run
   /* may be used by TBD RIO PID LOOP 
    SmartDashboard.putNumber("Left Front Goal RPMs", modules[0].RPMGoal);
    SmartDashboard.putNumber("Right Front Goal RPMs", modules[1].RPMGoal);
    SmartDashboard.putNumber("Left Back Goal RPMs", modules[2].RPMGoal);
    SmartDashboard.putNumber("Right Back Goal RPMs", modules[3].RPMGoal);

    SmartDashboard.putNumber("Left Front Goal Angle", modules[0].angleGoal);
    SmartDashboard.putNumber("Right Front Goal Angle", modules[1].angleGoal);
    SmartDashboard.putNumber("Left Back Goal Angle", modules[2].angleGoal);
    SmartDashboard.putNumber("Right Back Goal Angle", modules[3].angleGoal);

    SmartDashboard.putNumber("Left Front AngleMotor Output", modules[0].angleMotorOutput);
    SmartDashboard.putNumber("Right Front AngleMotor Output", modules[1].angleMotorOutput);
    SmartDashboard.putNumber("Left Back AngleMotor Outpute", modules[2].angleMotorOutput);
    SmartDashboard.putNumber("Right Back AngleMotor Output", modules[3].angleMotorOutput);

    SmartDashboard.putNumber("Left Front AngleMotor Error", modules[0].angleError);
    SmartDashboard.putNumber("Right Front AngleMotor Error", modules[1].angleError);
    SmartDashboard.putNumber("Left Back AngleMotor Error", modules[2].angleError);
    SmartDashboard.putNumber("Right Back AngleMotor Error", modules[3].angleError);
    */

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    /**  TBD - if RIO PID loop is needed
    SmartDashboard.putNumber("Left Front Goal RPMs", modules[0].RPMGoal);
    SmartDashboard.putNumber("Right Front Goal RPMs", modules[1].RPMGoal);
    SmartDashboard.putNumber("Left Back Goal RPMs", modules[2].RPMGoal);
    SmartDashboard.putNumber("Right Back Goal RPMs", modules[3].RPMGoal);

    SmartDashboard.putNumber("Left Front Goal Angle", modules[0].angleGoal);
    SmartDashboard.putNumber("Right Front Goal Angle", modules[1].angleGoal);
    SmartDashboard.putNumber("Left Back Goal Angle", modules[2].angleGoal);
    SmartDashboard.putNumber("Right Back Goal Angle", modules[3].angleGoal);
    */
  }
}
