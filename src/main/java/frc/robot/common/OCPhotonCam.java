// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.auto.Paths.GSType;
import frc.robot.util.MathHelp;

/**
 * Wrapper for PhotonCamera that provides convenience methods.
 */
public class OCPhotonCam extends PhotonCamera{

    private double kCamHeight;
    private double kCamPitch;

    private double kTargetHeight;

    public OCPhotonCam(NetworkTable table){
        super(table);
    }
    public OCPhotonCam(String camName){
        super(camName);
    }
    public OCPhotonCam(String camName, double camHeightInches, double camPitchDegrees){
        this(camName);
        setCamHeight(camHeightInches);
        setCamPitch(camPitchDegrees);
    }
    public OCPhotonCam(String camName, double camHeightInches, double camPitchDegrees, double targetHeightInches){
        this(camName);
        setCamHeight(camHeightInches);
        setCamPitch(camPitchDegrees);
        setTargetHeight(targetHeightInches);
    }

    public double getCamHeight(){
        return kCamHeight;
    }
    public double getCamPitch(){
        return kCamPitch;
    }
    public double getTargetHeight(){
        return kTargetHeight;
    }
    public void setCamHeight(double heightInches){
        kCamHeight = Units.inchesToMeters(heightInches);
    }
    /**
     * Sets the camera pitch for pose estimation.
     * @param pitchDegrees 0 = camera perpindicular to horizontal
     */
    public void setCamPitch(double pitchDegrees){
        kCamPitch = Units.degreesToRadians(pitchDegrees);
    }
    public void setTargetHeight(double heightInches){
        kTargetHeight = Units.inchesToMeters(heightInches);
    }

    public double getBestDistance(){
        PhotonPipelineResult result = getLatestResult();
        if(!result.hasTargets()) return 0;
        PhotonTrackedTarget target = result.getBestTarget();
        return PhotonUtils.calculateDistanceToTargetMeters(
            kCamHeight, kTargetHeight, kCamPitch, Units.degreesToRadians(target.getPitch()));
    }

    public GSType findGSType(){
        PhotonPipelineResult result = getLatestResult();
        if(!result.hasTargets() || MathHelp.isBetweenBounds(2, result.getTargets().size(), 3)) return GSType.FAIL;
        List<PhotonTrackedTarget> targets = result.getTargets();

        double firstPitch = targets.get(0).getPitch();
        double secondYaw = targets.get(1).getYaw();
        // determine path based on powercell position
        if(firstPitch < -5){ // Red is close
            if(secondYaw > 5){ // Red B is right
                return GSType.RED_B;
            }
            else{ // Red A is left, but the second power cell might confuse
                return GSType.RED_A;
            }
        }
        else{ // Blue is far
             // in both paths latter powercells are to the left
            if(targets.size()==3){ // 3 more visible in A
                return GSType.BLUE_A;
            }
            else{
                return GSType.BLUE_B;
            }
        }
    }

    /**
     * Attempts to find a list of translations based on the visible targets.
     * This can be used to create a trajectory based sequentially on the camera's target ordering.
     * Requires target height. If no targets are seen, an empty list is returned.
     */
    public List<Translation2d> findTranslationsThroughTargets(){
        PhotonPipelineResult result = getLatestResult();
        if(!result.hasTargets()) return new ArrayList<>();
        List<PhotonTrackedTarget> targets = result.getTargets();
        List<Translation2d> translationList = new ArrayList<>();

        for(PhotonTrackedTarget target : targets){
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                kCamHeight, kTargetHeight, kCamPitch, Units.degreesToRadians(target.getPitch()));
            Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
                distance, Rotation2d.fromDegrees(-target.getYaw()));

            translationList.add(translation);
        }

        return translationList;
    }
}
