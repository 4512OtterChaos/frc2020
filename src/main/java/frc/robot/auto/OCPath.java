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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
 */
public class OCPath extends Trajectory{

    public enum Preset{
        DEFAULT,
        SLALOM,
        BOUNCE,
        BARREL
    }

    private TrajectoryConfig config; // Store config for reversing

    private List<Pose2d> poses = new ArrayList<>();

    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and quintic hermite splines.
     */
    public OCPath(List<Pose2d> poses, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        this(
            TrajectoryGenerator.generateTrajectory(
                poses,
                getPresetConfig(feedforward, kinematics)
            ),
            getPresetConfig(feedforward, kinematics)
        );
        this.poses = new ArrayList<>(poses);
    }
    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and quintic hermite splines.
     */
    public OCPath(List<Pose2d> poses, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Preset preset){
        this(
            TrajectoryGenerator.generateTrajectory(
                poses,
                getPresetConfig(preset, feedforward, kinematics)
            ),
            getPresetConfig(preset, feedforward, kinematics)
        );
        this.poses = new ArrayList<>(poses);
    }
    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and clamped cubic splines,
     * where the heading at the interior waypoints is automatically determined.
     */
    public OCPath(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        this(
            TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                getPresetConfig(feedforward, kinematics)
            ), 
            getPresetConfig(feedforward, kinematics)
        );
        poses.add(start);
        for(Translation2d tran : interiorWaypoints){
            poses.add(
                new Pose2d(tran, new Rotation2d()) // poses without rotation  
            );
        }
        poses.add(end);
    }
    /**
     * Constructs a new Trajectory using {@link TrajectoryGenerator} and clamped cubic splines,
     * where the heading at the interior waypoints is automatically determined.
     */
    public OCPath(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Preset preset){
        this(
            TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                getPresetConfig(preset, feedforward, kinematics)
            ), 
            getPresetConfig(preset, feedforward, kinematics)
        );
        poses.add(start);
        for(Translation2d tran : interiorWaypoints){
            poses.add(
                new Pose2d(tran, new Rotation2d()) // poses without rotation  
            );
        }
        poses.add(end);
    }
    /**
     * Constructs a new OCPath with given trajectory and configuration. This constructor should be
     * used when more specific control is required, either with a specific trajectory implementation or
     * with a manually defined configuration.
     */
    public OCPath(Trajectory trajectory, TrajectoryConfig config){
        this(trajectory.getStates(), config);
    }
    /**
     * Constructs a new OCPath with given states and configuration. This constructor should be
     * used when more specific control is required, either with a specific trajectory implementation or
     * with a manually defined configuration.
     */
    public OCPath(List<State> states, TrajectoryConfig config){
        super(states);        
        this.config = config;
    }

    /**
     * Gets one of the preset TrajectoryConfigs and constructs it with feedforward and kinematics.
     */
    public static TrajectoryConfig getPresetConfig(Preset preset, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        switch(preset){
            default: return new TrajectoryConfig(kMaxVelocityMeters, kMaxAccelerationMeters)
                .setKinematics(kinematics)
                .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)) // Take corners slow
                .addConstraint(new DifferentialDriveVoltageConstraint(feedforward, kinematics, kMaxAutoVoltage)); // Account for voltage sag
            case SLALOM: return new TrajectoryConfig(12.25, 16)
                .setKinematics(kinematics)
                .addConstraint(new CentripetalAccelerationConstraint(10.5))
                .addConstraint(new DifferentialDriveVoltageConstraint(feedforward, kinematics, 12));
            case BOUNCE: return new TrajectoryConfig(11, 14)
                .setKinematics(kinematics)
                .addConstraint(new CentripetalAccelerationConstraint(9))
                .addConstraint(new DifferentialDriveVoltageConstraint(feedforward, kinematics, 12));
        }
    }
    /**
     * Gets the default TrajectoryConfigs and constructs it with feedforward and kinematics.
     */
    public static TrajectoryConfig getPresetConfig(SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics){
        return getPresetConfig(Preset.DEFAULT, feedforward, kinematics);
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
    public List<Pose2d> getPoseList(){
        return new ArrayList<>(poses);
    }

    /**
     * Returns a clone of this OCPath, reversing the trajectory's states automatically.
     * Ex: Follow a trajectory with the robot facing backwards.
     */
    public OCPath getReversed(){
        return new OCPath(
            getReversedStates(getStates()),
            getReversedConfig(getConfig())
        );
    }
    /**
     * Returns a clone of this OCPath, inverting the trajectory to
     * be followed the end to start(backwards).
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
     * Returns a cloned, reversed list of the given poses(backwards).
     */
    public static List<Pose2d> getInvertedPoses(List<Pose2d> poses){
        List<Pose2d> reversedPoses = new ArrayList<Pose2d>(poses);
        Collections.reverse(reversedPoses);
        return reversedPoses;
    }
    public static List<State> getReversedStates(List<State> states){
        List<State> reversedstates = new ArrayList<State>(states);
        
        for(int i=0;i<reversedstates.size();i++){
            State currState =  reversedstates.get(i);
            State newState = new State(
                currState.timeSeconds, 
                currState.velocityMetersPerSecond *-1, 
                currState.accelerationMetersPerSecondSq *-1, 
                new Pose2d(currState.poseMeters.getTranslation(), currState.poseMeters.getRotation().plus(new Rotation2d(Math.PI))), 
                currState.curvatureRadPerMeter *-1);
            reversedstates.set(i, newState);
        }
        return reversedstates;
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
