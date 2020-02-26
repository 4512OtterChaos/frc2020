/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.common.Testable;
import static frc.robot.common.Constants.VisionConstants.*;

/**
 * Class for interfacing with a Limelight.
 */
public class Limelight implements Testable{
    public enum Configuration{
        DRIVE(1, 0, 2),
        BASIC(0, 1, 2),
        PNP(0, 2, 2);

        private int ledMode;
        private int pipeline;
        private int streamMode;

        private Configuration(int ledMode, int pipeline, int streamMode){
            this.ledMode = ledMode;
            this.pipeline = pipeline;
            this.streamMode = streamMode;
        }

        public int getLedMode(){
            return ledMode;
        }
        public int getPipeline(){
            return pipeline;
        }
        public int getStreamMode(){
            return streamMode;
        }
    }

    private NetworkTable visionTable;

    private Configuration currConfiguration;

    private static final int averageSampleSize = 3;
    private LinearFilter txFilter = LinearFilter.movingAverage(averageSampleSize);
    private LinearFilter tyFilter = LinearFilter.movingAverage(averageSampleSize);

    private Timer changeTimer = new Timer(); // block data values for a period after changing pipelines

    public Limelight(){
        this(Configuration.PNP);
    }
    public Limelight(Configuration state){
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        setConfiguration(state);
    }

    public void setConfiguration(Configuration configuration){
        currConfiguration = configuration;
        changeTimer.reset();
        changeTimer.start();
        setLedMode(configuration.getLedMode());
        setPipeline(configuration.getPipeline());
        setStreamMode(configuration.getStreamMode());
    }
    protected void setLedMode(int value){
        if(getLedMode() != value)
            visionTable.getEntry("ledMode").setDouble(value);
    }
    protected void setPipeline(int value){
        if(getPipeline() != value)
            visionTable.getEntry("pipeline").setDouble(value);
    }
    protected void setStreamMode(int value){
        if(getStreamMode() != value)
            visionTable.getEntry("stream").setDouble(value);
    }
    

    public Configuration getConfiguration(){
        return currConfiguration;
    }
    public double getLedMode(){
        return visionTable.getEntry("ledMode").getDouble(0);
    }
    public double getPipeline(){
        return visionTable.getEntry("getpipe").getDouble(0);
    }
    public double getStreamMode(){
        return visionTable.getEntry("stream").getDouble(0);
    }
    public boolean getHasTarget(){
        return visionTable.getEntry("tv").getDouble(0) != 0;
    }
    public double getTx(){
        return visionTable.getEntry("tx").getDouble(0);
    }
    public double getTy(){
        return visionTable.getEntry("ty").getDouble(0);
    }
    public double getArea(){
        return visionTable.getEntry("area").getDouble(0);
    }
    public double getLatencySeconds(){
        return (visionTable.getEntry("tl").getDouble(0)+kLatencyMs)/1000.0;
    }
    public boolean isConnected(){
        return visionTable.getEntry("tl").getDouble(0)!=0;
    }
    public double[] get3d(){
        double[] camtran = visionTable.getEntry("camtran").getDoubleArray(new double[]{});
        SmartDashboard.putNumberArray("camtran", camtran);
        return camtran;
    }
    public double getPNP_X(){
        return get3d()[2];
    }
    public double getPNP_Y(){
        return get3d()[0];
    }
    public double getPNP_Z(){
        return get3d()[1];
    }
    public double getPNP_Pitch(){
        return get3d()[3];
    }
    public double getPNP_Yaw(){
        return get3d()[4];
    }
    public double getPNP_Roll(){
        return get3d()[5];
    }

    //-- Vision localization
    /**
     * Returns robot pose by translation.
     * @param camPose Pose of the camera
     */
    public Pose2d getRelativeRobotPose(Pose2d camPose){
        Translation2d robotTran = camPose.getTranslation().minus(
            kCameraTranslation.rotateBy(camPose.getRotation())
        );
        return new Pose2d(robotTran, camPose.getRotation());
    }
    /**
     * Returns field pose based on relative robot pose to the outer port.
     */
    public Pose2d getFieldPose(Pose2d robotPose){
        return new Pose2d(
            kTargetTranslation.plus(robotPose.getTranslation()),
            robotPose.getRotation()
        );

    }

    // PNP
    /**
     * Estimates camera pose relative to the outer port with PNP.
     */
    private Pose2d getRelativeCamPose(){
        return new Pose2d(getPNP_X(), getPNP_Y(), new Rotation2d(Units.degreesToRadians(getPNP_Yaw())));
    }
    /**
     * Estimates robot pose relative to the outer port with PNP.
     */
    public Pose2d getRelativeRobotPose(){
        return getRelativeRobotPose(getRelativeCamPose());
    }
    
    /**
     * Estimates field pose based on relative robot pose to the outer port with PNP.
     */
    public Pose2d getFieldPose(){
        return getFieldPose(getRelativeRobotPose());
    }
    /**
     * Estimates distance in inches(from camera) to target based on relative PNP field position.
     */
    public double getPNPDistance(){
        Pose2d fieldPose = getFieldPose();
        Translation2d fieldTran = fieldPose.getTranslation().plus(
            kCameraTranslation.rotateBy(fieldPose.getRotation())
        );
        return Units.metersToInches(fieldTran.getDistance(kTargetTranslation));
    }
    
    // Trig
    /**
     * Estimates camera pose relative to the outer port based on angle and heading.
     */
    private Pose2d getRelativeCamPose(Rotation2d heading){
        double radians = heading.getRadians()+Math.PI*0.5;
        double metersDist = Units.inchesToMeters(getTrigDistance());
        double y = Math.cos(radians)*metersDist;
        double x = Math.sin(radians)*-metersDist;
        return new Pose2d(x, y, heading);
    }
    /**
     * Estimates robot pose relative to the outer port based on angle and heading.
     */
    public Pose2d getRelativeRobotPose(Rotation2d heading){
        return getRelativeRobotPose(getRelativeCamPose(heading));
    }
    /**
     * Estimates field pose based on relative robot pose with angle and heading.
     */
    public Pose2d getFieldPose(Rotation2d heading){
        return getFieldPose(getRelativeRobotPose(heading));
    }
    /**
     * Estimates distance in inches(from camera) to target based on angle.
     */
    public double getTrigDistance(){
        double difference = kTargetHeight-kCameraHeight;
        double angle = Units.degreesToRadians(kCameraAngle+getTy());
        if(!getHasTarget()) return 0;
        return (difference/Math.tan(angle));
    }

    public Pose2d getRelativeTargetPose(double heading){
        Pose2d targetPose = new Pose2d();
        return targetPose;
    }


    public double getFilteredTx(){
        if(!isBlocked())  return txFilter.calculate(getTx());
        else return 0;
    }
    public double getFilteredTy(){
        if(!isBlocked()) return tyFilter.calculate(getTy());
        else return 0;
    }

    private boolean isBlocked(){
        if(changeTimer.get()>0.2){
            changeTimer.stop();
            return false;
        }
        return false;
    }

    public void log(){
        SmartDashboard.putNumber("Trig Distance", getTrigDistance());
        SmartDashboard.putNumber("PNP Distance", getPNPDistance());
        SmartDashboard.putNumber("PNP X", getRelativeCamPose().getTranslation().getX());
        SmartDashboard.putString("PNP Pose", getFieldPose().toString());
    }

    @Override
    public TestableResult test(){
        Status result = isConnected() ? Status.PASSED : Status.FAILED;
        return new TestableResult("Limelight", result);
    }
    
}
