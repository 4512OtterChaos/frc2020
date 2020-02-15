/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.common.Constants.Vision.*;

/**
 * Class for interfacing with a Limelight.
 */
public class Limelight implements Loggable{
    public enum State{
        DRIVE(1, 0, 2),
        BASIC(0, 1, 1),
        PNP(0, 2, 1);

        private int ledMode;
        private int pipeline;
        private int streamMode;

        private State(int ledMode, int pipeline, int streamMode){
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

    private State currState;

    private static final int averageSampleSize = 4;
    private LinearFilter txFilter = LinearFilter.movingAverage(averageSampleSize);
    private LinearFilter tyFilter = LinearFilter.movingAverage(averageSampleSize);

    private Timer changeTimer = new Timer(); // block data values for a period after changing pipelines

    public Limelight(){
        this(State.PNP);
    }
    public Limelight(State state){
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        setState(state);
    }

    public void setState(State state){
        currState = state;
        changeTimer.reset();
        changeTimer.start();
        setLedMode(state.getLedMode());
        setPipeline(state.getPipeline());
        setStreamMode(state.getStreamMode());
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
    

    public State getState(){
        return currState;
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
    @Log
    public boolean getHasTarget(){
        return visionTable.getEntry("tv").getDouble(0) != 0;
    }
    @Log
    public double getTx(){
        return visionTable.getEntry("tx").getDouble(0);
    }
    @Log
    public double getTy(){
        return visionTable.getEntry("ty").getDouble(0);
    }
    public double getArea(){
        return visionTable.getEntry("area").getDouble(0);
    }
    public double getLatencySeconds(){
        return (visionTable.getEntry("tl").getDouble(0)+kLatencyMs)/1000.0;
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
    public Pose2d getRelativeCamPose(){
        return new Pose2d(getPNP_X(), getPNP_Y(), new Rotation2d(Units.degreesToRadians(getPNP_Yaw())));
    }
    public Pose2d getRelativeRobotPose(){
        return getRelativeCamPose();
    }

    @Log
    public double getFilteredTx(){
        if(!isBlocked())  return txFilter.calculate(getTx());
        else return 0;
    }
    @Log
    public double getFilteredTy(){
        if(!isBlocked()) return tyFilter.calculate(getTy());
        else return 0;
    }
    /*
    @Config
    public static void setCamAngle(double angle){
        camAngle = angle;
    }*/
    @Log
    public double getTrigDistance(){
        double difference = kTargetHeight-kCameraHeight;
        double angle = Units.degreesToRadians(kCameraAngle+getTy());
        return (difference/Math.tan(angle));
    }

    private boolean isBlocked(){
        if(changeTimer.get()>0.2){
            changeTimer.stop();
            return false;
        }
        return false;
    }
    
}
