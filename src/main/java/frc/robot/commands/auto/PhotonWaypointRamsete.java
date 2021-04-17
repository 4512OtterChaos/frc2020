// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.OCPath;
import frc.robot.auto.OCPath.Preset;
import frc.robot.common.OCPhotonCam;
import frc.robot.common.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class PhotonWaypointRamsete extends CommandBase {

    private OCPhotonCam camera;
    private Drivetrain drivetrain;
    private StandardRamseteCommand ramseteCommand;

    public PhotonWaypointRamsete(OCPhotonCam camera, Drivetrain drivetrain) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        List<Translation2d> waypoints = camera.findTranslationsThroughTargets();

        if(waypoints.isEmpty()){
            end(false);
            return;
        }

        Pose2d startPose = new Pose2d(
            Units.feetToMeters(2.5), Units.feetToMeters(9.125), new Rotation2d()
        );
        drivetrain.resetOdometry(startPose);
        List<Translation2d> relativeWaypoints = new ArrayList<>();
        for(Translation2d waypoint : waypoints){
            relativeWaypoints.add(
                waypoint.rotateBy(startPose.getRotation()).plus(startPose.getTranslation())
                    .minus(new Translation2d(0.25,0)) // quick fix turning
            );
        }

        NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        
        List<Pose2d> ballPoses = relativeWaypoints.stream().map(
            translation -> new Pose2d(translation, new Rotation2d())
        ).collect(Collectors.toList());
        liveTable.getEntry("visionTargets").setValue(ballPoses.toArray());

        //Translation2d lastTran = relativeWaypoints.get(relativeWaypoints.size()-1);
        //Pose2d endPose = new Pose2d(new Translation2d(Units.feetToMeters(32), lastTran.getY()), new Rotation2d());

        // use the last waypoint as the end pose with 0 heading
        Pose2d endPose = new Pose2d(relativeWaypoints.get(relativeWaypoints.size()-1), new Rotation2d());
        relativeWaypoints.remove(relativeWaypoints.size()-1);

        OCPath path = new OCPath(
            startPose, 
            relativeWaypoints,
            endPose, 
            drivetrain.getLinearFF(), drivetrain.getKinematics(), Preset.GALACTIC
        );

        ramseteCommand = new StandardRamseteCommand(drivetrain, path);
        ramseteCommand.initialize();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(ramseteCommand != null) ramseteCommand.execute();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(ramseteCommand != null) ramseteCommand.end(interrupted);
    }
}
