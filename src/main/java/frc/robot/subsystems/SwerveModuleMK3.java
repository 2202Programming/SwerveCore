package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotMap;


public class SwerveModuleMK3 {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation (neo has 42 ticks per rotation)
  //private static double kEncoderTicksPerRotation = 4096; //cancoder
  private static double kEncoderTicksPerRotation = 42; //neo built-in

  private CANSparkMax driveMotor;
  private CANSparkMax angleMotor;

  private CANPIDController driveMotorPID;
  private CANPIDController angleMotorPID; //sparkmax PID can only use internal NEO encoders
  private PIDController anglePID; //roborio PID so we can use CANCoders

  private CANCoder canCoder;

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

    angleMotorPID.setP(kAngleP);
    angleMotorPID.setI(kAngleI);
    angleMotorPID.setD(kAngleD);
    angleMotorPID.setFeedbackDevice(angleMotor.getEncoder());

    anglePID = new PIDController(kAngleP, kAngleI, kAngleD);

    driveMotorPID.setP(kDriveP);
    driveMotorPID.setI(kDriveI);
    driveMotorPID.setD(kDriveD);
    driveMotorPID.setFF(kDriveF);
    driveMotorPID.setFeedbackDevice(driveMotor.getEncoder());

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);

  }


  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sesnor value reports degrees.
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()); //for cancoder
    //return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition()/360.0); //built-in encoder returns rotations?  convert rotation to degrees
  }

  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity(); //in RPM
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle(); //in degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new rotational position
    //Rotation2d rotationDelta = state.angle.minus(currentRotation);

    // Find the new absolute position of the module based on the difference in rotation
    //double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    //double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    //double desiredTicks = currentTicks + deltaTicks;
    angleGoal = state.angle.getDegrees();
    angleError = angleGoal - currentRotation.getDegrees();
    //angleMotorPID.setReference(angleGoal/360, ControlType.kPosition); //setReference wants rotations
    angleMotorOutput = MathUtil.clamp(anglePID.calculate(getAngle().getDegrees(),angleGoal),-RobotMap.MAX_ANGLE_MOTOR_OUTPUT,RobotMap.MAX_ANGLE_MOTOR_OUTPUT);
    angleMotor.set(angleMotorOutput); //roborio PID for angle, clamping  max output

    double feetPerSecondGoal = Units.metersToFeet(state.speedMetersPerSecond);
    RPMGoal = (feetPerSecondGoal*60)/(Math.PI * RobotMap.WHEEL_DIAMETER); //convert feet per sec to RPM goal
    driveMotorPID.setReference(RPMGoal, ControlType.kVelocity); //wants RPM
  }

}
