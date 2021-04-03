// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.OCPath;
import frc.robot.common.OCPhotonCam;
import frc.robot.subsystems.Drivetrain;

public class PhotonWaypointRamsete extends CommandBase {

    private OCPhotonCam camera;
    private Drivetrain drivetrain;
    private StandardRamseteCommand ramseteCommand;

    public PhotonWaypointRamsete(OCPhotonCam camera, Drivetrain drivetrain) {
        this.camera = camera;
        this.drivetrain = drivetrain;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        List<Translation2d> waypoints = camera.findTranslationsThroughTargets();

        if(waypoints.size()==0) end(false);

        Pose2d startPose = drivetrain.getOdometry().getPoseMeters();
        List<Translation2d> relativeWaypoints = waypoints.stream().map( // make waypoints relative to starting pose since they are technically transforms
            translation -> translation.rotateBy(startPose.getRotation()).plus(startPose.getTranslation())
        ).collect(Collectors.toList());
        
        // use the last waypoint as the end pose with 0 heading
        Pose2d endPose = new Pose2d(relativeWaypoints.get(relativeWaypoints.size()-1), new Rotation2d());
        relativeWaypoints.remove(relativeWaypoints.size()-1);

        ramseteCommand = new StandardRamseteCommand(drivetrain,
            new OCPath(
                startPose, 
                relativeWaypoints,
                endPose, 
                drivetrain.getLinearFF(), drivetrain.getKinematics(), false
            )
        );
        ramseteCommand.initialize();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ramseteCommand.execute();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
    }
}
