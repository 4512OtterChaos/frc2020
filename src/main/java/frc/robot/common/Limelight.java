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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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

    private State currState = State.DRIVE;

    private LinearFilter txFilter = LinearFilter.movingAverage(3);
    private LinearFilter tyFilter = LinearFilter.movingAverage(3);

    private Timer changeTimer = new Timer(); // block data values for a period after changing pipelines

    public Limelight(){
        this(State.DRIVE);
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
        return visionTable.getEntry("pipeline").getDouble(0);
    }
    public double getStreamMode(){
        return visionTable.getEntry("stream").getDouble(0);
    }
    @Log
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
    public Pose2d getPose(){
        return new Pose2d();
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

    private boolean isBlocked(){
        if(changeTimer.get()>0.2){
            changeTimer.stop();
            return false;
        }
        return false;
    }
    
}
