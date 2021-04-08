/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

    /**
     * CAN bus IDs
     * 
     * Please keep in order ID order
     * 
     */
    public static final class CAN {
        // CAN ID for non-motor devices
        public static final int PDP = 0; // this must be 0
        public static final int PCM1 = 1; // default ID for PCM
        public static final int PCM2 = 2;

        // drive train CANCoders
        public static final int DT_BL_CANCODER = 5;
        public static final int DT_BR_CANCODER = 6; 
        public static final int DT_FR_CANCODER = 7;
        public static final int DT_FL_CANCODER = 8;
        
        // drive train drive / angle motors - sparkmax neo
        public static final int DT_FL_DRIVE = 20;
        public static final int DT_FL_ANGLE = 21;

        public static final int DT_BL_DRIVE = 22;
        public static final int DT_BL_ANGLE = 23;

        public static final int DT_BR_DRIVE = 24; 
        public static final int DT_BR_ANGLE = 25; 

        public static final int DT_FR_DRIVE = 26; 
        public static final int DT_FR_ANGLE = 27;

        
        

    }

    // PWM assignments on the Rio
    public static final class PWM {
      /*
      public static final int INTAKE = 0;
      public static final int MAGAZINE = 9; 
      */
    }
    
    // Digital IO on the RIO
    public static final class DigitalIO {
      /*
      public static final int LEFT_CHASSIS_ENCODER_A = 0;
      public static final int LEFT_CHASSIS_ENCODER_B = 1;
      public static final int MAGAZINE_GATE = 2;  
      public static final int MAGAZINE_GATE_PWR = 4;  
      public static final int RIGHT_CHASSIS_ENCODER_A = 5;
      public static final int RIGHT_CHASSIS_ENCODER_B = 6;
      */
    }


    // Analog IO on the RIO
    public static final class AnalogIn {
       // public static final int MAGAZINE_ANGLE = 0;
    }

    //Pnumatics control 2 -
    public static final class PCM2 {
      //public static final int MAG_LOCK = 0;
      //public static final int MAG_UNLOCK = 1;
    }

 
    public static final class RobotPhysical {
        public static final double BUMPER_TO_LIDAR = 100; // mm 
        public static final double LIDAR_TO_LIDAR = 348;  // mm 

        //useful if we do modeling for tracking
        public static final double Mass = 145;  // lbs with battery and code loaded

        //chassis  
        public static final double WheelDiameter = 6.0; // inches, nominal
        public static final double WheelAxleDistance = 2.22418; //(robot char 3/20/21)    //25.5/12.0; // feet 25.5 mid-wheel, 26.75 outer wheel

        //wheel wear compensation - adjust when distance is off by small amount
        public static final double WheelWearLeft = 0.99;   //[percent] of nominal
        public static final double WheelWearRight = 0.99;  //[percent] of nominal
        
    }

    /**
     * Subsystem constants
     * 
     * Maybe be used directly by the subsystem or passed as args in construction 
     * depending on the need.
     * 
     *    <subsys>.data  convention 
     */
    public static final class LIDAR {
        public static final double SAMPLE_mS = 20; // in ms
       
    }

    public static final class DriverPrefs {
        public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
        public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
        public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
    }

    public static final class DriveTrain {
        // motor constraints
        public static final double motorMaxRPM = 5600;    // motor limit
        public static final double wheelDiameter = 0.5; //[ft]
        /****
        // Other constraints
        public static final int smartCurrentMax = 60;  //amps in SparkMax, max setting
        public static final int smartCurrentLimit = 35; //amps in SparkMax, inital setting
        */
        // Acceleration limits
        public static final double slewRateMax = 2;      //sec limits adjusting slewrate 
        public static final boolean safetyEnabled = false; 
    }

  
  
}
