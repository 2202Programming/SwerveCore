package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveTrain;


public class SwerveModuleMK3 {

  //mk3 gear ratios
  final double MAX_ANGLE_MOTOR_OUTPUT = 0.1;   // [0.0 to 1.0] 
  final double kSteeringGR = 12.8;             // [mo-turns to 1 angle wheel turn]
  final double kDriveGR = 8.16;                // [mo-turn to 1 drive wheel turn]

  final int kSlot = 0;    // using slot 0 for angle and drive pid on SmartMax

  //Hardware PID settings in Constants.DriveTrain PIDFController

  //Software PID - TBD
  private static final double kAngleP = 0.001;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // devices
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final CANPIDController driveMotorPID;
  private final CANPIDController angleMotorPID; // sparkmax PID can only use internal NEO encoders
  private final CANEncoder  angleEncoder;
  private final CANEncoder  driveEncoder;

  private final PIDController anglePID;         // roborio PID so we can use CANCoders -TBD

  private final CANCoder canCoder;

  //for debugging
  CANCoderConfiguration canCoderConfiguration;

  public double angleGoal;
  public double RPMGoal;
  public double angleMotorOutput;
  public double angleError;
  public double internalAngle;

  public SwerveModuleMK3(CANSparkMax driveMotor, CANSparkMax angleMotor, Rotation2d offset, CANCoder canCoder, boolean invertAngle, boolean invertDrive) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;

    angleMotor.setInverted(invertAngle);
    driveMotor.setInverted(invertDrive);

    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);

    driveMotorPID = driveMotor.getPIDController();
    angleMotorPID = angleMotor.getPIDController();
    angleEncoder = angleMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    //set angle endcoder to return values in deg and deg/s
    angleEncoder.setPositionConversionFactor(360.0/kSteeringGR);        // mo-rotations to degrees
    angleEncoder.setVelocityConversionFactor(360.0/kSteeringGR/60.0);   // rpm to deg/s
   
    //set driveEncoder to use ft/s
    driveEncoder.setPositionConversionFactor(Math.PI*DriveTrain.wheelDiameter/kDriveGR);      // mo-rotations to ft
    driveEncoder.setPositionConversionFactor(Math.PI*DriveTrain.wheelDiameter/kDriveGR/60.0); // mo-rpm to ft/s

    // Pid around CANCoder angle to assist angleMotor interal PID - TBD
    // DPL - maybe use this to close error after calibration?  
    anglePID = new PIDController(kAngleP, kAngleI, kAngleD);
    anglePID.enableContinuousInput(-180.0, 180.0);              // -180 == +180

    // SparkMax PID values
    DriveTrain.anglePIDF.copyTo(angleMotorPID, kSlot);          // position mode 
    DriveTrain.drivePIDF.copyTo(driveMotorPID, kSlot);          // velocity mode

    calibrate(offset.getDegrees());
  }

  void calibrate(double offsetDegrees) {

    // adjust magnetic offset in CANCoder, measured constants.
    canCoderConfiguration = new CANCoderConfiguration();
    canCoder.getAllConfigs(canCoderConfiguration);             // read existing settings (debug)
    canCoderConfiguration.magnetOffsetDegrees = offsetDegrees; // correct offset
    canCoder.configMagnetOffset(offsetDegrees);                // update corrected offset
    
    // now read canCoder position
    double pos_deg = canCoder.getAbsolutePosition();
    // set to absolute starting angle of CANCoder
    angleEncoder.setPosition(pos_deg);     
    var angle = angleEncoder.getPosition();
    anglePID.reset();
    anglePID.calculate(pos_deg, pos_deg);
  }

  public void periodic() {
    internalAngle = angleEncoder.getPosition();
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
    // uses the motor's internal position (not absolute, but calibrated at power up)
    return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition());
  }

  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity(); // ft/s
  }

  //sets a -180 to 180 paradigm for angle
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
    SwerveModuleState state = desiredState;//SwerveModuleState.optimize(desiredState, currentRotation);
    internalAngle = angleEncoder.getPosition();
    // use position control on angle with INTERNAL encoder, scaled internally for degrees
    angleMotorPID.setReference(state.angle.getDegrees(), ControlType.kPosition);

    // use velocity control, internally scales for ft/s.
    double feetPerSecondGoal = Units.metersToFeet(state.speedMetersPerSecond);
    feetPerSecondGoal = 0.0;
    driveMotorPID.setReference(feetPerSecondGoal, ControlType.kVelocity); 
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

    // set the velocity of the drive, scaled to use ft/s
    double feetPerSecondGoal = Units.metersToFeet(state.speedMetersPerSecond);
    feetPerSecondGoal = 0.0;
    driveMotorPID.setReference(feetPerSecondGoal, ControlType.kVelocity);
  }



}