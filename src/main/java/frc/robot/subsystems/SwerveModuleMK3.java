package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;


public class SwerveModuleMK3 {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private CANSparkMax driveMotor;
  private CANSparkMax angleMotor;
  private CANCoder canCoder;

  private CANPIDController driveMotorPID;
  private CANPIDController angleMotorPID;
  

  public SwerveModuleMK3(CANSparkMax driveMotor, CANSparkMax angleMotor, CANCoder canCoder, Rotation2d offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;

    driveMotorPID = driveMotor.getPIDController();
    angleMotorPID = angleMotor.getPIDController();

    angleMotorPID.setP(kAngleP);
    angleMotorPID.setI(kAngleI);
    angleMotorPID.setD(kAngleD);
    angleMotorPID.setFeedbackDevice(angleMotor.getEncoder());

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
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / SwerveDrivetrain.kMaxSpeed);
  }

}
