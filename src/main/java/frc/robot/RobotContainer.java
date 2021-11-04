// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverPrefs;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static RobotContainer  rc;
  public static RobotContainer   RC() {return rc;}

  public final HID_Xbox_Subsystem driverControls;
  public final Sensors_Subsystem sensors;
  private final SwerveDrivetrain drivetrain;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.rc = this;

    //create our subsystems
    sensors = new Sensors_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone); 
    drivetrain = new SwerveDrivetrain();

    // set default commands
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverControls));

    
  }

  // testing 
  void test_controls() {
    // var cmd = new MK3_AngleSpeed(driverControls, drivetrain, 0);  // FL, FR, BL, BR (0..3)
    //drivetrain.setDefaultCommand(cmd);

    //testing commands, speed in feet per sec, angle in degrees
    //driverControls.bind(Id.Driver, XboxPOV.POV_UP).whenHeld(new SwerveDriveTest(drivetrain, 1, 0));
    //driverControls.bind(Id.Driver, XboxPOV.POV_RIGHT).whenHeld(new SwerveDriveTest(drivetrain, 1, 90));
    //driverControls.bind(Id.Driver, XboxPOV.POV_DOWN).whenHeld(new SwerveDriveTest(drivetrain, 1, 180));
    //driverControls.bind(Id.Driver, XboxPOV.POV_LEFT).whenHeld(new SwerveDriveTest(drivetrain, 1, -90));
  }
}
