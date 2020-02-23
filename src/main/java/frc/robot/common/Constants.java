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

    public static final double kRobotDelta = 0.01;

    public static final double kFieldWidth = Units.inchesToMeters(323.75);
    public static final double kFieldDepth = Units.inchesToMeters(629.25);
    
    // All distance measurements should be in meters when being used
    
    public static class DrivetrainConstants{
        public static final double kTrackWidthMeters = Units.inchesToMeters(23); // Distance between center of drivetrain sides
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);

        public static final double kMaxVelocityMeters = Units.feetToMeters(14);
        public static final double kMaxVelocityRadians = Units.degreesToRadians(400);
        
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation

        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 75;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }

    public static class ShooterConstants{
        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 80;

        public static final double kRampRaw = 0.04; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    public static class ShooterWristConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 30;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;

        public static final int kVelocityConstraint = 7000; // Motor RPM
        public static final int kAccelerationConstraint = 10000; // Motor RPM per second

        public static final double kEncoderOffset = -0.35; // Add this to encoder value to make "0" flat

        // Constraints
        public static final double kMinDownwardRotations = 0.04;
        public static final double kLowerSafeRotations = 0.075; // above is safe
        public static final double kHigherSafeRotations = 0.175; // below is safe
        public static final double kBufferRotations = 0.015; // amount to clear "safe" bounds by
    }

    public static class IntakeConstants{
        public static final int kFreeLimit = 22;
        public static final int kStallLimit = 27;

        public static final double kRampRaw = 0.4; // seconds to full output(on the motor)
    }
    public static class IntakeArmConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 32;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;

        public static final double kEncoderOffset = -0.05; // Add this to encoder value to make "0" flat

        // Constraints
        public static final double kLowerSafeRotations = 0.075; // below is safe
        public static final double kHigherSafeRotations = 0.175; // above is safe
        public static final double kBufferRotations = 0.015; // amount to clear "safe" bounds by
        public static final double kMaxUpwardRotations = 0.245; // 90 degrees up
        public static final int kVelocityConstraint = 7000; // Motor RPM
        public static final int kAccelerationConstraint = 10000; // Motor RPM per second
    }

    public static class IndexerConstants{
        public static final int kFreeLimit = 22;
        public static final int kStallLimit = 27;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }

    public static class LiftConstants{
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation

        public static final int kFreeLimit = 27;
        public static final int kStallLimit = 35;

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
        public static final int kMaxHeightRotations = 100; // Max height in motor rotations
        public static final int kVelocityConstraint = 7000; // Motor RPM
        public static final int kAccelerationConstraint = 10000; // Motor RPM per second
    }

    public static class VisionConstants{
        public static final double kCameraAngle = 30; // Angle of camera offset from horizontal
        public static final double kCameraHeight = 24; // inches
        public static final double kTargetHeight = 98.25;
        public static final double kLatencyMs = 11; // Image capture latency
        public static final Translation2d kCameraTranslation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        public static final Translation2d kTargetTranslation = new Translation2d(kFieldDepth, Units.inchesToMeters(94.66));
    }
    
    public static class AutoConstants{
        public static final double kMaxVelocityMeters = Units.feetToMeters(10);
        public static final double kMaxAccelerationMeters = Units.feetToMeters(6);
        public static final double kMaxCentripetalAccelerationMeters = Units.feetToMeters(3.4); // Turning acceleration given radius
        public static final Pose2d kAcceptablePoseError = new Pose2d(0.8, 0.8, new Rotation2d(Units.degreesToRadians(60))); // this is for crash handling on trajectories
        public static final double kReferenceFailureWindow = 1.75; // seconds till trajectory abandons when off tolerence
    }
}
