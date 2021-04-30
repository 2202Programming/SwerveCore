// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;

public class CharecterizationSubsytem extends SubsystemBase {
  /** Creates a new CharecterizationSubsytem. */

  static private double ENCODER_EDGES_PER_REV = 1;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 1;
  static private double GEARING = 8.16;
  
  private double encoderConstant = (1 / GEARING);

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  String data = "";
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;
  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  SwerveDrivetrain drive;

  public CharecterizationSubsytem(SwerveDrivetrain drivetrain) {
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
    drive = drivetrain;

    //Use front modules to represent left and right sides
    rightEncoderPosition = ()
    -> drive.getMK3(1).getDriveEncoder().getPosition() * encoderConstant;
    rightEncoderRate = ()
    -> drive.getMK3(1).getDriveEncoder().getVelocity() * encoderConstant / 60.;
    leftEncoderPosition = ()
    -> drive.getMK3(0).getDriveEncoder().getPosition() * encoderConstant;
    leftEncoderRate = ()
    -> drive.getMK3(0).getDriveEncoder().getVelocity() * encoderConstant / 60.;

    gyroAngleRadians = ()
    -> Math.toRadians(drive.getGryoHeading());

    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    data = "";

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables, should be between -1 and 1.
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    if (rotateEntry.getBoolean(false)){
      //convert autospeed to rad/s as a % of max angular speed
      drive.drive(0, 0, autospeed*DriveTrain.kMaxAngularSpeed, false); //rotation only mode.  
    } else {
      //convert autospeed to ft/s as a % of max linear speed
      drive.testDrive(autospeed*DriveTrain.kMaxSpeed ,0); //straight travel mode
    }

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
