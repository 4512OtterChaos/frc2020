/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Helper methods for field dimensions / transforms.
 */
public final class FieldUtil {

    /**
     * Returns pose with (x,y) based off of heading and distance.
     * @param heading Robot heading on field
     * @param distMeters Distance in meters from target robot is relating to
     */
    public static Pose2d getRelativePose(Rotation2d heading, double distMeters){
        double radians = heading.getRadians()+Math.PI*0.5;
        double y = Math.cos(radians)*distMeters;
        double x = Math.sin(radians)*-distMeters;
        return new Pose2d(x, y, heading);
    }

    /**
     * Returns the rotation that points the robot at the target if robot heading = 9
     */
    public static Rotation2d getRelativeHeading(Translation2d robotTran, Translation2d targetTran){
        double x = targetTran.getX() - robotTran.getX();
        double y = targetTran.getY() - robotTran.getY();
        return new Rotation2d(Math.atan2(y, x));
    }
    /**
     * Returns the rotation that points the robot at the target
     */
    public static Rotation2d getRelativeHeading(Pose2d robotPose, Translation2d targetTran){
        Rotation2d theta = getRelativeHeading(robotPose.getTranslation(), targetTran);
        return theta.minus(robotPose.getRotation());
    }
    public static Rotation2d getTargetedHeading(Pose2d robotPose, Translation2d targetTran){
        return robotPose.getRotation().plus(getRelativeHeading(robotPose, targetTran));
    }
}
