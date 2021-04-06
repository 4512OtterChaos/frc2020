/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
/**
 * Holds autonomous trajectories and methods.
 */
public class Paths {

    private static NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");

    //----- Paths
    public final OCPath example; // our different paths
    public final OCPath slalom;
    public final OCPath bounce1;
    public final OCPath bounce2;
    public final OCPath bounce3;
    public final OCPath bounce4;
    //-----

    private static boolean hasAbandonedTrajectory = false;

    /**
     * Generates autonomous paths given drivetrain specifications.
     */
    public Paths(SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        example = new OCPath(PathsList.example, feedforward, kinematics);
        slalom = quinticToCubic(PathsList.slalom, feedforward, kinematics);
        bounce1 = new OCPath(PathsList.bounce1, feedforward, kinematics).getReversed();
        bounce2 = new OCPath(PathsList.bounce2, feedforward, kinematics);
        bounce3 = new OCPath(PathsList.bounce3, feedforward, kinematics).getReversed();
        bounce4 = new OCPath(PathsList.bounce4, feedforward, kinematics);
    }

    public static OCPath quinticToCubic(List<Pose2d> poses, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        List<Translation2d> interior = new ArrayList<>();
        for(Pose2d pose : poses){ // create interior waypoints
            interior.add(new Translation2d(pose.getX(), pose.getY()));
        }
        interior.remove(0); // trim start/end poses
        interior.remove(interior.size()-1);
        return new OCPath(
            poses.get(0),
            interior,
            poses.get(poses.size()-1),
            feedforward, kinematics
        );
    }

    /**
     * Lists pose lists for different paths.
     * We explicitly hard-code these waypoints because it is easier to modify/tune quickly--
     * if you would like a visual representation, use FalconDashboard.
     */
    public static class PathsList{
        // All pose distance measurements are recorded in feet(but translated to meters)!
        public static final List<Pose2d> example = feetToMeters(// dumb
            new Pose2d(),
            new Pose2d(4, 2, new Rotation2d()),
            new Pose2d(8, 0, new Rotation2d())
        );
        public static final List<Pose2d> slalom = Arrays.asList(// clamped cubic
            //--good values--
            // velocity: 12.25
            // accel: 16
            // centripetal: 10.5
            // sag: 12
            toPose(3.1, 2, 30), // start
            toPose(7.5, 5, 50),
            toPose(10.75, 7.3, 0),
            toPose(19.5, 7.1,  0),
            toPose(22.5, 5, -60),
            toPose(25, 2.75, 0),
            toPose(27, 5, 90),
            toPose(25, 7.3, 0),
            toPose(22.5, 5, -130),
            toPose(20, 2.9, 0),
            toPose(15, 3, 180),
            toPose(9, 3.5, 0),
            toPose(6.75, 6, 0),
            toPose(2.5, 10, 140)
        );
        public static final List<Pose2d> bounce1 = Arrays.asList(
            // reversed
            toPose(3.5, 7.5, 0),
            toPose(7.5, 11.75, 90)  
        );
        public static final List<Pose2d> bounce2 = Arrays.asList(
            // forward
            toPose(7.5, 11.5, -90),
            toPose(10, 5, -70),
            toPose(15, 5, 90),
            toPose(15, 11, 90)
        );
        public static final List<Pose2d> bounce3 = Arrays.asList(
            // reversed
            toPose(15, 11, -90),
            toPose(15, 5, -90),
            toPose(22.5, 5, 90),
            toPose(22.5, 12.5, 90)  
        );
        public static final List<Pose2d> bounce4 = Arrays.asList(
            toPose(22.5, 12.5, -80),
            toPose(27, 6.75, -30)
        );
        
        
        public static Pose2d toPose(double xFt, double yFt, double deg){
            return new Pose2d(Units.feetToMeters(xFt),Units.feetToMeters(yFt),Rotation2d.fromDegrees(deg));
        }
        public static Translation2d toTran(double xFt, double yFt){
            return new Translation2d(Units.feetToMeters(xFt), Units.feetToMeters(yFt));
        }
        /**
         * Takes pose waypoints in feet, returning the list as poses in meters.
         */
        public static List<Pose2d> feetToMeters(Pose2d... poses){
            return Arrays.asList(poses).stream()
                .map(pose -> new Pose2d(new Translation2d(
                    Units.feetToMeters(pose.getTranslation().getX()),
                    Units.feetToMeters(pose.getTranslation().getY())),
                    pose.getRotation()))
                .collect(Collectors.toList());
        }
        /**
         * Takes pose waypoints in meters, returning the list as poses in feet
         * (Should only be used for telemetry).
         */
        public static List<Pose2d> metersToFeet(Pose2d... poses){
            return Arrays.asList(poses).stream()
                .map(pose -> new Pose2d(new Translation2d(
                    Units.metersToFeet(pose.getTranslation().getX()),
                    Units.metersToFeet(pose.getTranslation().getY())),
                    pose.getRotation()))
                .collect(Collectors.toList());
        }
    }

    /**
     * Returns true if the last trajectory followed was abandoned due to being outside acceptable tolerance for too long.
     * Resets on new trajectory initialization.
     * Since this method is static, it is useful as a global flag for interrupting autonomous command groups.
     */
    public static boolean getHasAbandonedTrajectory(){
        SmartDashboard.putBoolean("Abandoned Trajectory", hasAbandonedTrajectory);
        return hasAbandonedTrajectory;
    }
    /**
     * Sets a flag that a trajectory was abandoned while being followed, interrupting current auto command.
     */
    public static void abandonTrajectory(){
        hasAbandonedTrajectory = true;
    }

    /**
     * Logs an instantaneous point in a trajectory to the live visualizer.
     * @param trajectory Trajectory to track
     * @param timeSeconds Seconds at which pose is sampled (e.g. time since trajectory has started)
     */
    public static void logTrajectory(Trajectory trajectory, double timeSeconds){
        Pose2d currPose = trajectory.sample(timeSeconds).poseMeters; // current pose
        liveTable.getEntry("pathX").setDouble(Units.metersToFeet(currPose.getTranslation().getX()));
        liveTable.getEntry("pathY").setDouble(Units.metersToFeet(currPose.getTranslation().getY()));
        liveTable.getEntry("isFollowingPath").setBoolean(timeSeconds <= trajectory.getTotalTimeSeconds());
    }
}
