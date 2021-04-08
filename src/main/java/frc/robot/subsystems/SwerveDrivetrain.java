// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  /**
   * TODO: These are example values and will need to be adjusted for your robot!
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
      Units.inchesToMeters(10),
      Units.inchesToMeters(10)
    ),
    new Translation2d(
      Units.inchesToMeters(10),
      Units.inchesToMeters(-10)
    ),
    new Translation2d(
      Units.inchesToMeters(-10),
      Units.inchesToMeters(10)
    ),
    new Translation2d(
      Units.inchesToMeters(-10),
      Units.inchesToMeters(-10)
    )
  );

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  
  private SwerveModuleMK3[] modules = new SwerveModuleMK3[] {
    new SwerveModuleMK3(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), Rotation2d.fromDegrees(0), new CANCoder(RobotMap.DRIVETRAIN_FRONT_LEFT_ENCODER)), // Front Left
    new SwerveModuleMK3(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), Rotation2d.fromDegrees(0), new CANCoder(RobotMap.DRIVETRAIN_FRONT_RIGHT_ENCODER)), // Front Right
    new SwerveModuleMK3(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), Rotation2d.fromDegrees(0), new CANCoder(RobotMap.DRIVETRAIN_BACK_LEFT_ENCODER)), // Back Left
    new SwerveModuleMK3(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, 
      CANSparkMax.MotorType.kBrushless), Rotation2d.fromDegrees(0), new CANCoder(RobotMap.DRIVETRAIN_BACK_RIGHT_ENCODER))  // Back Right
  };

  public SwerveDrivetrain() {
    gyro.reset(); 
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front RPMs", modules[0].getVelocity());
    SmartDashboard.putNumber("Right Front RPMs", modules[1].getVelocity());
    SmartDashboard.putNumber("Left Back RPMs", modules[2].getVelocity());
    SmartDashboard.putNumber("Right Back RPMs", modules[3].getVelocity());

    SmartDashboard.putNumber("Left Front Angle", modules[0].getAngle().getDegrees());
    SmartDashboard.putNumber("Right Front Angle", modules[1].getAngle().getDegrees());
    SmartDashboard.putNumber("Left Back Angle", modules[2].getAngle().getDegrees());
    SmartDashboard.putNumber("Right Back Angle", modules[3].getAngle().getDegrees());

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

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    SmartDashboard.putNumber("Left Front RPMs", modules[0].getVelocity());
    SmartDashboard.putNumber("Right Front RPMs", modules[1].getVelocity());
    SmartDashboard.putNumber("Left Back RPMs", modules[2].getVelocity());
    SmartDashboard.putNumber("Right Back RPMs", modules[3].getVelocity());

    SmartDashboard.putNumber("Left Front Angle", modules[0].getAngle().getDegrees());
    SmartDashboard.putNumber("Right Front Angle", modules[1].getAngle().getDegrees());
    SmartDashboard.putNumber("Left Back Angle", modules[2].getAngle().getDegrees());
    SmartDashboard.putNumber("Right Back Angle", modules[3].getAngle().getDegrees());

    SmartDashboard.putNumber("Left Front Goal RPMs", modules[0].RPMGoal);
    SmartDashboard.putNumber("Right Front Goal RPMs", modules[1].RPMGoal);
    SmartDashboard.putNumber("Left Back Goal RPMs", modules[2].RPMGoal);
    SmartDashboard.putNumber("Right Back Goal RPMs", modules[3].RPMGoal);

    SmartDashboard.putNumber("Left Front Goal Angle", modules[0].angleGoal);
    SmartDashboard.putNumber("Right Front Goal Angle", modules[1].angleGoal);
    SmartDashboard.putNumber("Left Back Goal Angle", modules[2].angleGoal);
    SmartDashboard.putNumber("Right Back Goal Angle", modules[3].angleGoal);

  }
}
