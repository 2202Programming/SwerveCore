package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveTrain;


public class SwerveModuleMK3 {
  public final String NT_Name = "DT";  //expose data under DriveTrain table

  // PID slot for angle and drive pid on SmartMax controller
  final int kSlot = 0;    

  // Hardware PID settings in Constants.DriveTrain PIDFController 

  // Rev devices
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;
  private final CANPIDController driveMotorPID;
  private final CANPIDController angleMotorPID; // sparkmax PID can only use internal NEO encoders
  private final CANEncoder  angleEncoder;       // aka internalAngle
  private final CANEncoder  driveEncoder;
  //CTRE devices
  private final CANCoder absEncoder;            // aka externalAngle (external to Neo/Smartmax)
  private double angleCmdInvert;

  // Software PID - TBD unused-TBD if needed
  // private final PIDController anglePID = new PIDController(0.001, 0.0, 0.0);
  // final double MAX_ANGLE_MOTOR_OUTPUT = 0.1; // [0.0 to 1.0]

  /**
   * Warning CANCoder and CANEncoder are very close in name but very different.
   * 
   * CANCoder: CTRE, absolute position mode, +/- 180 CCW= positive CANEncoder:
   * RevRobotics, relative position only, must configure to CCW based on side &
   * gearing Continous positon so postion can be greater than 180 because it can
   * "infinitely" rotate. Cannot be inverted in Brushless mode, must invert motor
   * 
   */
  // for debugging
  CANCoderConfiguration absEncoderConfiguration;

  /**
   * TBD if we need PID on RIO public double angleGoal; public double RPMGoal;
   * public double angleMotorOutput; public double angleError;
   **/

  // NetworkTables
  String NTPrefix;

  // measurements made every period - public so they can be pulled for network
  // tables...
  public double m_internalAngle;
  public double m_externalAngle;
  public double m_velocity;

  public SwerveModuleMK3(CANSparkMax driveMtr, CANSparkMax angleMtr, double offsetDegrees, CANCoder absEnc,
      boolean invertAngleMtr, boolean invertAngleCmd, boolean invertDrive) {
    driveMotor = driveMtr;
    angleMotor = angleMtr;
    absEncoder = absEnc;

    // account for command sign differences if needed
    _setInvertAngleCmd(invertAngleCmd);

    // Drive Motor config
    driveMotor.setInverted(invertDrive);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotorPID = driveMotor.getPIDController();
    driveEncoder = driveMotor.getEncoder();
    // set driveEncoder to use ft/s
    driveEncoder.setPositionConversionFactor(Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR); // mo-rotations
                                                                                                        // to ft
    driveEncoder.setVelocityConversionFactor(Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR / 60.0); // mo-rpm
                                                                                                               // to
                                                                                                               // ft/s

    // Angle Motor config
    angleMotor.setInverted(invertAngleMtr);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotorPID = angleMotor.getPIDController();
    angleEncoder = angleMotor.getEncoder();

    // set angle endcoder to return values in deg and deg/s
    angleEncoder.setPositionConversionFactor(360.0 / DriveTrain.kSteeringGR); // mo-rotations to degrees
    angleEncoder.setVelocityConversionFactor(360.0 / DriveTrain.kSteeringGR / 60.0); // rpm to deg/s

    // Pid around absEncoder angle to assist angleMotor interal PID - TBD
    // DPL - maybe use this to close error after calibration?
    // anglePID.enableContinuousInput(-180.0, 180.0); // -180 == +180

    // SparkMax PID values
    DriveTrain.anglePIDF.copyTo(angleMotorPID, kSlot); // position mode
    DriveTrain.drivePIDF.copyTo(driveMotorPID, kSlot); // velocity mode

    calibrate(offsetDegrees);
  }

  void calibrate(double offsetDegrees) {

    // adjust magnetic offset in absEncoder, measured constants.
    absEncoderConfiguration = new CANCoderConfiguration();
    absEncoder.getAllConfigs(absEncoderConfiguration); // read existing settings (debug)
    absEncoderConfiguration.magnetOffsetDegrees = offsetDegrees; // correct offset
    absEncoder.configMagnetOffset(offsetDegrees); // update corrected offset

    // now read absEncoder position
    double pos_deg = absEncoder.getAbsolutePosition();
    // set to absolute starting angle of absEncoder
    angleEncoder.setPosition(pos_deg);
    // anglePID.reset();
    // anglePID.calculate(pos_deg, pos_deg);
  }

  // _set<>  for testing during bring up.
  public void _setInvertAngleCmd(boolean invert) {
    angleCmdInvert = (invert) ? -1.0 : 1.0;
  }
  public void _setInvertAngleMotor(boolean invert) {
    angleMotor.setInverted(invert);
  }
  public void _setInvertDriveMotor(boolean invert) {
    driveMotor.setInverted(invert);
  }
  /**
   *  setNTPrefix - causes the network table entries to be created 
   *  and updated on the periodic() call.
   * 
   *  Use a short string to indicate which MK unit this is.
   * 
   */
  public SwerveModuleMK3 setNTPrefix(String prefix) {
    NTPrefix = "/MK3-" + prefix;
    NTConfig();
    return this;
  }

  public String getNTPrefix() { 
    return NTPrefix; 
  }

  public void periodic() {
    //measure everything at same time
    m_internalAngle = angleEncoder.getPosition()*angleCmdInvert;
    m_externalAngle = absEncoder.getAbsolutePosition();
    m_velocity = driveEncoder.getVelocity();

    NTUpdate();
  }

  /**
   * This is the angle being controlled, so it should be thought of as the real
   * angle of the wheel.
   * 
   * @return SmartMax/Neo internal angle (degrees)
   */
  public Rotation2d getAngleRot2d() {
    return Rotation2d.fromDegrees(m_internalAngle); // for cancoder
  }
  public double getAngle() {
    return m_internalAngle;
  }


  /**
   * External Angle is external to the SmartMax/Neo and is the absolute 
   * angle encoder. 
   * 
   * At power-up, this angle is used to calibrate the SmartMax PID controller.
   * 
   */
  public Rotation2d getAngleExternalRot2d() {
    return Rotation2d.fromDegrees(m_externalAngle);
  }
  public double getAngleExternal() {
    return m_externalAngle;
  }

  /**
   * 
   * @return velocity (ft/s)
   */
  public double getVelocity() {
    return m_velocity;
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
    SwerveModuleState state = desiredState;//SwerveModuleState.optimize(desiredState, currentRotation);

    // use position control on angle with INTERNAL encoder, scaled internally for degrees
    angleMotorPID.setReference(angleCmdInvert * state.angle.getDegrees(), ControlType.kPosition);

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
  /**  TBD
  void testing_setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngleRot2d();
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
  ***/

  /**
   * Network Tables data 
   * 
   * If a prefix is given for the module, NT entries will be created and updated on the periodic() call.
   * 
   */
  private NetworkTable table;
  private NetworkTableEntry nte_angle;
  private NetworkTableEntry nte_external_angle;
  private NetworkTableEntry nte_velocity;

  void NTConfig() {
    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nte_angle = table.getEntry(NTPrefix + "/angle");
    nte_external_angle = table.getEntry(NTPrefix +"/angle_ext");
    nte_velocity = table.getEntry(NTPrefix + "/velocity");
  }

  void NTUpdate() {
    if (table == null) return;                   // not initialized, punt
    nte_angle.setDouble(m_internalAngle);
    nte_external_angle.setDouble(m_externalAngle);
    nte_velocity.setDouble(m_velocity);
  }

}