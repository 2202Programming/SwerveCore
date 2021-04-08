package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveTrain;


public class SwerveModuleMK3 {

  // TODO: move to Constants at some point
  final double MAX_ANGLE_MOTOR_OUTPUT = 0.1;   //[0.0 to 1.0] 

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 0.001;
  // private static final double kDriveI = 0.01;
  // private static final double kDriveD = 0.1;
  // private static final double kDriveF = 0.2;
  private static final double kDriveI = 0.0;
  private static final double kDriveD = 0.0;
  private static final double kDriveF = 0.0;

  private static final double kAngleP = 0.001;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation (neo has 42 ticks per rotation)
  // private static double kEncoderTicksPerRotation = 4096; //cancoder
  private static double kEncoderTicksPerRotation = 42; // neo built-in

  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final CANPIDController driveMotorPID;
  private final CANPIDController angleMotorPID; // sparkmax PID can only use internal NEO encoders
  private final CANEncoder  angleEncoder;
  private final CANEncoder  driveEncoder;

  private final PIDController anglePID; // roborio PID so we can use CANCoders

  private final CANCoder canCoder;

  public double angleGoal;
  public double RPMGoal;
  public double angleMotorOutput;
  public double angleError;

  public SwerveModuleMK3(CANSparkMax driveMotor, CANSparkMax angleMotor, Rotation2d offset, CANCoder canCoder) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;

    driveMotorPID = driveMotor.getPIDController();
    angleMotorPID = angleMotor.getPIDController();
    angleEncoder = angleMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    // SparkMax Angle motor/encoder - position mode should be used
    angleMotorPID.setP(kAngleP);
    angleMotorPID.setI(kAngleI);
    angleMotorPID.setD(kAngleD);
    angleMotorPID.setFeedbackDevice(angleMotor.getEncoder());   // this tells Angle motor to use it's  internal Encoder


    //TODO: set all the scale factors to use degrees or radians and (ft/ or m/s) on encoders

    //pid around CANCoder angle to assist angleMotor interal PID
    // DPL - maybe use this to close error after calibration?  not sure
    anglePID = new PIDController(kAngleP, kAngleI, kAngleD);
    anglePID.enableContinuousInput(0.0, 360.0);                // 0 and 360 should be same point

    // SparkMax Motor and encoder - velocity mode should be used
    driveMotorPID.setP(kDriveP);
    driveMotorPID.setI(kDriveI);
    driveMotorPID.setD(kDriveD);
    driveMotorPID.setFF(kDriveF);
    driveMotorPID.setFeedbackDevice(driveMotor.getEncoder());

    /****
     * 
     * DPL  not sure of below code and how offset should be used.
     * CANCoder have been setup in sensors, so we should be able to read 
     * an absolute position and use that to calibrate the Neo positon.
     * 
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);
    **/
    calibrate();
  }

  void calibrate() {
    Rotation2d pos = getAngle();
    double pos_deg = pos.getDegrees();
    angleEncoder.setPosition(pos_deg);     // sets internal angle to current measure absolute angle
    anglePID.reset();
    anglePID.calculate(pos_deg, pos_deg);

  }


  /**
   * Gets the relative rotational position of the module
   * 
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback
    // coefficient and the sesnor value reports degrees.
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()); // for cancoder
    // return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition()/360.0);
    // //built-in encoder returns rotations? convert rotation to degrees
  }

  public Rotation2d getAngleInternal() {
    //TODO: check scaling, /360 looks wrong to me (DPL)
    // uses the motor's internal position (not absolute)
    return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition()/360.0);
  }

  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity(); // in RPM
  }

  public double angleFix(double angle) {
    if (angle > 180) {
      return angle - 360;
    } else {
      return angle;
    }
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * 
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle(); 
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // use position control on angle with INTERNAL encoder
    // setReference wants rotations by default - TODO - check the scaling
    angleMotorPID.setReference(state.angle.getDegrees()/360, ControlType.kPosition);

    double feetPerSecondGoal = Units.metersToFeet(state.speedMetersPerSecond);
    RPMGoal = (feetPerSecondGoal * 60) / (Math.PI *DriveTrain.wheelDiameter); // convert feet per sec to RPM goal
    driveMotorPID.setReference(feetPerSecondGoal, ControlType.kVelocity); // wants RPM
  }

  /**
   * The code below will use a RIO pid loop around the CANCode and use the 
   * angleMotor in velocity mode.
   * 
   */
  void testing_setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle(); // (CANCoder)
    // Find the difference between our current rotational position + our new
    // rotational position
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
  
    // use a PID loop around simple %motor output
    double angleGoal = currentRotation.getDegrees();
    double angleCmd = anglePID.calculate(currentRotation.getDegrees(), angleGoal);
  
    angleMotorOutput = MathUtil.clamp( angleCmd, -MAX_ANGLE_MOTOR_OUTPUT, MAX_ANGLE_MOTOR_OUTPUT);
    angleMotor.set(angleMotorOutput); // roborio PID for angle, clamping max output

    // set the velocity of the drive
    double feetPerSecondGoal = Units.metersToFeet(state.speedMetersPerSecond);
    RPMGoal = (feetPerSecondGoal * 60) / (Math.PI * DriveTrain.wheelDiameter); // convert feet per sec to RPM goal
    driveMotorPID.setReference(feetPerSecondGoal, ControlType.kVelocity); // wants RPM

  }



}