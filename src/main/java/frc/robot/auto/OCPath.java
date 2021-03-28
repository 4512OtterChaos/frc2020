/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static frc.robot.common.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * Extension of the {@link Trajectory} class that simplifies the construction and use of Trajectories.
 * Trajectories are constructed by default with a preset configuration.
 * To retrieve a trajectory in reverse, simply use .getReversed()
 */
public class OCPath extends Trajectory{
    private TrajectoryConfig config; // Store config for reversing

    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and quintic hermite splines.
     * Drivetrain is given to use a default config with its kinematics.
     */
    public OCPath(List<Pose2d> poses, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, boolean reversed){
        this(
            TrajectoryGenerator.generateTrajectory(
                poses,
                getDefaultConfig(feedforward, kinematics).setReversed(reversed)
            ).getStates(),
            getDefaultConfig(feedforward, kinematics).setReversed(reversed)
        );
    }
    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and clamped cubic splines,
     * where the heading at the interior waypoints is automatically determined.
     * Drivetrain is given to use a default config with its kinematics.
     */
    public OCPath(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, boolean reversed){
        this(
            TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                getDefaultConfig(feedforward, kinematics).setReversed(reversed)
            ).getStates(), 
            getDefaultConfig(feedforward, kinematics).setReversed(reversed)
        );
    }
    /**
     * Constructs a new Trajectory with given states and configuration. This constructor should be
     * used when more specific control is required, either with a specific trajectory implementation or
     * with a manually defined configuration.
     */
    public OCPath(List<State> states, TrajectoryConfig config){
        super(states);        
        this.config = config;
    }

    /**
     * Returns a clone of the given {@link TrajectoryConfig}.
     */
    public static TrajectoryConfig cloneConfig(TrajectoryConfig config){
        return new TrajectoryConfig(config.getMaxVelocity(), config.getMaxAcceleration())
            .addConstraints(config.getConstraints())
            .setReversed(config.isReversed());
    }

    /**
     * Returns the default configuration to be used for generating trajectories.
     * @param drive Drivetrain to specify kinematics
     */
    public static TrajectoryConfig getDefaultConfig(SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        return new TrajectoryConfig(kMaxVelocityMeters, kMaxAccelerationMeters)
            .setKinematics(kinematics)
            .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)) // Take corners slow
            .addConstraint(new DifferentialDriveVoltageConstraint(feedforward, kinematics, 10)); // Account for voltage sag
    }
    /**
     * Returns a cloned list of Trajectory states.
     */
    public List<State> getStates(){
        return new ArrayList<State>(super.getStates());
    }
    /**
     * Returns a clone of this Trajectory's configuration.
     */
    public TrajectoryConfig getConfig(){
        return cloneConfig(config);
    }

    public Pose2d getInitialPose(){
        return getStates().get(0).poseMeters;
    }

    /**
     * Returns a clone of this OCPath, inverting the trajectory to
     * be followed the opposite direction.
     * Ex: Follow a trajectory, and then follow the same trajectory inverted
     * to return to the starting point.
     */
    public OCPath getInverted(){
        return new OCPath(
            getInvertedStates(getStates()), 
            getReversedConfig(getConfig())
        );
    }
    /**
     * Returns a cloned, reversed list of the given poses.
     */
    public static List<Pose2d> getReversedPoses(List<Pose2d> poses){
        List<Pose2d> reversedPoses = new ArrayList<Pose2d>(poses);
        Collections.reverse(reversedPoses);
        return reversedPoses;
    }
    /**
     * Returns a cloned, reversed list of the given states.
     * Each state is reconstructed with negative velocity and
     * the correct timepoint for following backwards.
     */
    public static List<State> getInvertedStates(List<State> states){
        List<State> invertedstates = new ArrayList<State>(states);
        Collections.reverse(invertedstates);
        
        for(int i=0;i<invertedstates.size();i++){
            State currState =  invertedstates.get(i);
            State newState = new State(
                states.get(i).timeSeconds, 
                currState.velocityMetersPerSecond *-1, 
                currState.accelerationMetersPerSecondSq, 
                currState.poseMeters, 
                currState.curvatureRadPerMeter);
            invertedstates.set(i, newState);
        }
        return invertedstates;
    }
    /**
     * Returns a clone of the given {@link TrajectoryConfig} with the reverse flag set to true.
     */
    public static TrajectoryConfig getReversedConfig(TrajectoryConfig config){
        return cloneConfig(config)
            .setReversed(!config.isReversed());
    }

}
