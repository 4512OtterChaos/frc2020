/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static final double kRobotDelta = 0.01;
    
    // All distance measurements should be in meters when being used
    
    public static class Drivetrain{
        public static final double kTrackWidthMeters = Units.inchesToMeters(23); // Distance between center of drivetrain sides
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);

        public static final double kMaxVelocityMeters = Units.feetToMeters(8);
        
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation

        public static final int kDriveStallCurrentLimit = 70;
        public static final int kDriveFreeCurrentLimit = 35;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    
    public static class Autonomous{
        public static final double kMaxVelocityMeters = Units.feetToMeters(10);
        public static final double kMaxAccelerationMeters = Units.feetToMeters(6);
        public static final double kMaxRadiansLowGear = Units.degreesToRadians(450); // Rotations per second
        public static final double kMaxCentripetalAccelerationMeters = Units.feetToMeters(3.4); // Turning acceleration given radius
    }
}
