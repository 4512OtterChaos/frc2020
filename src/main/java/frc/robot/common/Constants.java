/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    
    public static final double kRobotDelta = 0.02;
    
    public static final double kFieldWidth = Units.inchesToMeters(323.75);
    public static final double kFieldDepth = Units.inchesToMeters(629.25);
    
    // All distance measurements should be in meters when being used
    
    public static class DrivetrainConstants{
        public static final double kTrackWidthMeters = Units.inchesToMeters(23); // Distance between center of drivetrain sides
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        
        public static final double kMaxVelocityMeters = Units.feetToMeters(14);
        public static final double kMaxVelocityRadians = Units.degreesToRadians(700); // actual = ~500 degrees
        
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation
        
        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 75;
        
        public static final double kRampRaw = 0.04; // seconds to full output(on the motor)
        
        public static final double kLinearStaticFF = 0.241; // volts (Given from the characterization tool)
        public static final double kLinearVelocityFF = 2.32; // per meters per second
        public static final double kLinearAccelerationFF = 0.376; // per meters per second squared
        
        public static final double kAngularStaticFF = 0.261; // volts (Given from the characterization tool)
        public static final double kAngularVelocityFF = 2.4; // per meters per second (per side when turning)
        public static final double kAngularAccelerationFF = 0.49; // per meters per second squared
        
        public static final double kP = 0.8; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    
    public static class ShooterConstants{
        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 50;
        
        public static final double kRampRaw = 0.06; // seconds to full output(on the motor)
        
        public static final double klStaticFF = 0.483; // volts (Given from the characterization tool)
        public static final double klVelocityFF = 0.13; // per meters per second
        public static final double klAccelerationFF = 0; // per meters per second squared
        
        public static final double krStaticFF = 0.609; // volts (Given from the characterization tool)
        public static final double krVelocityFF = 0.13; // per meters per second
        public static final double krAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0.0005; // PID Gains
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    public static class ShooterWristConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 30;
        
        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kCounterGravityFF = 0.12;
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0.22; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
        
        public static final int kVelocityConstraint = 120; // Cruise Velocity Degrees
        public static final int kAccelerationConstraint = 160;
        
        public static final double kEncoderOffset = -0.36; // Add this to encoder value to make "0" flat
        
        // Constraints
        public static final double kLowestSafeDegrees = 10; // above is safe
        public static final double kClearIntakeDegrees = 30;
        public static final double kHighestSafeDegrees = 64; // below is safe
        public static final double kBufferDegrees = 1; // amount to clear "safe" bounds by
    }
    
    public static class IntakeConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 32;
        
        public static final double kRampRaw = 0.12; // seconds to full output(on the motor)
        
        public static final double kEncoderOffset = -0.17; // Add this to encoder value to make "0" flat
        
        // Constraints
        public static final double kEngagedDegrees = 20; // below is when arm is "down"
    }
    public static class IntakeSlideConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 32;
        
        public static final double kRampRaw = 0.15; // seconds to full output(on the motor)
    }
    
    public static class IndexerConstants{
        public static final int kFreeLimit = 38;
        public static final int kStallLimit = 45;
        
        public static final double kRampRaw = 0.04; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    
    public static class LiftConstants{
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation
        
        public static final int kFreeLimit = 35;
        public static final int kStallLimit = 37;
        
        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kLiftWeightCounter = 0;
        public static final double kRobotWeightCounterFF = 0;
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
        
        // Constraints
        public static final int kMaxHeightRotations = 265; // Max height in motor rotations
        public static final int kMinHeightRotations = -85; // Min height while climbing
        public static final int kVelocityConstraint = 7000; // Motor RPM
        public static final int kAccelerationConstraint = 10000; // Motor RPM per second
    }
    
    public static class VisionConstants{
        public static final double kShootPitch = 30; // Vertical angle of camera offset from 
        public static final double kIntakePitch = -20;
        public static final double kShootHeight = 26.75; // inches
        public static final double kIntakeHeight = 29.75;
        public static final double kTargetHeight = 98.25;
        public static final double kLatencyMsLime = 11; // Image capture latency
        public static final Translation2d kTranslation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        public static final Translation2d kTargetTranslation = new Translation2d(kFieldDepth, Units.inchesToMeters(94.66));
    }
    
    public static class AutoConstants{
        public static final double kMaxAutoVoltage = 10.5; // Avoid voltage sag and maintain accuracy
        public static final double kMaxVelocityMeters = Units.feetToMeters(12);
        public static final double kMaxAccelerationMeters = Units.feetToMeters(7);
        public static final double kMaxCentripetalAccelerationMeters = Units.feetToMeters(3.7); // Turning acceleration given radius
        public static final Pose2d kAcceptablePoseError = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(60))); // this is for crash handling on trajectories
        public static final double kReferenceFailureWindow = 1.75; // seconds till trajectory abandons when off tolerence
    }
}
