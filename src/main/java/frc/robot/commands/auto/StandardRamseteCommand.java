/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.RamseteController;
import frc.robot.auto.RamseteCommand;
import static frc.robot.common.Constants.*;
import static frc.robot.common.Constants.AutoConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import frc.robot.subsystems.Drivetrain;
import frc.robot.auto.OCPath;
import frc.robot.auto.Paths;

/**
 * A basic example of using ramsete controller to follow a short path
 */
public class StandardRamseteCommand extends RamseteCommand{

    private final Drivetrain drivetrain;
    private final OCPath trajectory;
    private final static RamseteController controller = new RamseteController(); // static is fine as most of the functionality is in calculate()
    private Timer timer = new Timer();
    private double lastTime;
    private double timeOffReference = 0;

    /**
     * Construct ramsete command using drivetrain and following given trajectory
     */
    public StandardRamseteCommand(Drivetrain drivetrain, OCPath trajectory){
        super(
            trajectory,
            () -> drivetrain.getOdometry().getPoseMeters(),
            controller,
            drivetrain.getLinearFF(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftController(),
            drivetrain.getRightController(),
            drivetrain::tankDriveVolts,
            drivetrain
        );

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        controller.setTolerance(kAcceptablePoseError);
    }

    @Override
    public void initialize(){
        super.initialize();

        lastTime = Timer.getFPGATimestamp();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        super.execute();

        double dt = Timer.getFPGATimestamp()-lastTime;
        if(!controller.atReference()){
            timeOffReference += dt;
        }
        else{
            timeOffReference = Math.max(0, timeOffReference-3*dt);
        }
        if(timeOffReference>kReferenceFailureWindow){
            Paths.abandonTrajectory();
            end(true);
        }
        lastTime = Timer.getFPGATimestamp();

        Paths.logTrajectory(trajectory, timer.get());
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);

        Paths.logTrajectory(trajectory, trajectory.getTotalTimeSeconds()+1);
        timer.stop();
        drivetrain.tankDrive(0, 0);
    }

    public RamseteController getController(){
        return controller;
    }
    public OCPath getPath(){
        return trajectory;
    }

    public static Command photonWaypointRamsete(Drivetrain drivetrain, PhotonCamera camera){
        if(!camera.hasTargets()) return new InstantCommand();
        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
        final double camHeight = Units.inchesToMeters(VisionConstants.kIntakeHeight);
        final double camPitch = Units.degreesToRadians(VisionConstants.kIntakePitch);
        final double targHeight = Units.inchesToMeters(3.5);
        List<Pose2d> waypoints = new ArrayList<>();

        for(int i=0;i<targets.size();i++){
            PhotonTrackedTarget target = targets.get(i);
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                camHeight, targHeight, camPitch, target.getPitch());
            Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
                distance, Rotation2d.fromDegrees(-target.getYaw()));
            Rotation2d heading = new Rotation2d();
            /*
            if(i+1>=targets.size()){
                double x = targets.get(i+1).getX() - target.getX();
                double y = targets.get(i+1).getY() - target.getY();
                heading = new Rotation2d(Math.atan2(y, x));
            }
            else{
                heading = new Rotation2d();
            }*/
            waypoints.add(
                new Pose2d(translation, heading)
            );
        }

        return new StandardRamseteCommand(drivetrain,
            new OCPath(
                waypoints, drivetrain.getLinearFF(), drivetrain.getKinematics(), false
            )
        );
    }
}
