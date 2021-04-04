// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.common.OCLEDManager.Configuration;
import frc.robot.util.MathHelp;

/**
 * Class for specifying unique LED patterns accessible on different strip lengths and configurations.
 */
public class LEDPattern {

    private final OCLEDManager manager; // manager of the LED strip and patterns
    protected final AddressableLEDBuffer buffer; // final buffer representing the whole LED strip
    protected final DoubleSupplier workingVal; // supplied double for variable patterns(e.g. progress bar)
    protected int length; // effective length of the strip(note: half buffer length when in split-configuration) used in patterns
    
    public LEDPattern(OCLEDManager manager){
        this(manager, ()->0);
    }
    public LEDPattern(OCLEDManager manager, DoubleSupplier workingVal){
        this.manager = manager;
        this.buffer = new AddressableLEDBuffer(manager.getTotalStripLength());
        this.length = manager.getConfiguredStripLength();
        this.workingVal = workingVal;
    }
    protected LEDPattern(LEDPattern other){
        this(other.manager, other.workingVal);
    }

    /**
     * A conditional pattern. If condition is true, draw the other pattern, otherwise draw this pattern.
     */
    public static LEDPattern conditionalWith(LEDPattern main, LEDPattern other, BooleanSupplier condition){
        return new LEDPattern(main){
            @Override
            public AddressableLEDBuffer draw(){
                boolean isOther = condition.getAsBoolean();
                if(isOther) return other.draw();
                else return main.draw();
            }
        };
    }

    /**
     * Draws this LEDPattern's pattern to its buffer and returns it.
     * This must be overridden with a specific pattern to be used.
     */
    public AddressableLEDBuffer draw(){
        return buffer;
    }

    /**
     * Writes HSV values to pixels in the buffer.
     * If in a split-configuration, it writing starts from the ends and meets in the middle.
     * @param i Pixel index
     * @param hue
     * @param sat
     * @param value
     */
    protected void setHSV(int i, int hue, int sat, int val) {
        buffer.setHSV(i, hue, sat, val);
        if(manager.getConfig() == Configuration.MIRRORSPLIT) buffer.setHSV(length*2-1-i, hue, sat, val);
        else if(manager.getConfig() == Configuration.COPYSPLIT) buffer.setHSV(length+i, hue, sat, val);
        /*
        for(int j=length*2;j<buffer.getLength();j++){
            boolean isRed = DriverStation.getInstance().getAlliance()==Alliance.Red;
            if(isRed) buffer.setHSV(j, red, 255, 200);
            else buffer.setHSV(j, blue, 255, 200);
        }
        */
    }

    // ----Helper methods
    /**
    * Finds the continuous distance between two pixel indexes.
    * 
    * @param a
    * @param b
    * @param length Effective length considering configuration
    * @return
    */
    protected static int findDifference(int a, int b, int length) {
        return Math.abs(Math.abs(a - b + length / 2) % length - length / 2);
    }
    /**
     * Interpolate between two HSV values
     * @param h1 From hue
     * @param s1 From saturation
     * @param v1 From value
     * @param h2 To hue
     * @param s2 To saturation
     * @param v2 To value
     * @param percent (0 - 1)
     * @return Interpolated {@link Color} object
     */
    protected static Color lerpHSV(int h1, int s1, int v1, int h2, int s2, int v2, double percent){
        int hue = (int)MathHelp.lerp(percent, h1, h2);
        int sat = (int)MathHelp.lerp(percent, s1, s2);
        int val = (int)MathHelp.lerp(percent, v1, v2);
        return Color.fromHSV(hue, sat, val);
    }

    // -----Constants affecting appearance
    public static final int kGreenHue = 60;
    public static final int kRedHue = 0;
    public static final int kBlueHue = 108;
    public static final int kYellowHue = 27;
    
    public static final int kWaveLength = 16; // size of waves
    public static final int kWaveThreshold = 100; // minimum wave brightness(value)

    // Persistent variables to change over time
    protected double workingHue = 0;
    protected double workingSat = 0;

    protected boolean hasBumperLighting = false;
    
    protected double lastTime = Timer.getFPGATimestamp();


    // -----Basic patterns
    // These methods are intended to be used internally for creating patterns.

    /**
    * Sets all pixels to HSV
    */
    protected void solid(int hue, int sat, int val) {
        for (int i = 0; i < length; i++) {
            setHSV(i, hue, sat, val);
        }
    }
    /**
    * Pulses the strip with hue and saturation at speed
    */
    protected void pulsing(int hue, int saturation, int speed) {
        int value = (int) (((Math.sin(Timer.getFPGATimestamp() * speed)) + 1) * (255 / 2.0));
        for (int i = 0; i < length; i++) {
            setHSV(i, hue, saturation, value);
        }
    }
    /**
    * Fills strip with pixels of hue to percent amount
    */
    protected void progressBar(int hue, double percentage) {
        MathHelp.clamp(percentage, 0, 1);
        solid(0, 0, 0); // make sure index values we dont touch are clear
        for (int i = 0; i < (int) (length * percentage); i++) {
            setHSV(i, hue, 255, 255);
        }
    }
    /**
    * Draws a moving dashed line(clear background).
    * 
    * @param gap Pixels between dashes
    * @param speed Pixels per second
    * @param size Pixel length of dashes
    */
    protected void dashes(int hue, int gap, int speed, int size){
        int offset = (int) (Timer.getFPGATimestamp() * speed % length);
        for (int i = 0; i < length; i++){
            int currIndex = (i + offset) % length;
            int currSize = i % size*2;

            if(currSize < size/2){ // dash vs clear background
                setHSV(currIndex, hue, 255, 255);
            }
            else{
                setHSV(currIndex, 0, 0, 0);
            }
        }
    }
    /**
    * Creates a moving wave that scrolls along the strip.
    * Note: call this method multiple times to create more waves in a strip.
    * 
    * @param initOffset Starting pixel index(useful with multiple waves)
    * @param speed Pixels per second
    * @param waveLength Pixel length of the wave
    * @param waveThreshold Minimum value of brightness outside of the wave
    */
    protected void wave(int hue, int sat, int initOffset, int speed, int waveLength, int waveThreshold){
        int offset = (int) (Timer.getFPGATimestamp() * speed % length + initOffset);
        for (int i = 0; i < length; i++){
            int difference = findDifference(offset, i, length);
            int value = (int) (255 - difference * (2 * (255 - waveThreshold) / (waveLength)));
            value = Math.max(waveThreshold, value);
            if(value > waveThreshold) setHSV(i, hue, sat, value);
        }
    }

    
    // -----Presets
    // After creating a blank pattern we can use these common presets to override the drawing method.

    public LEDPattern presetSolid(int hue, int sat, int val) {
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                solid(hue, sat, val);
                return buffer;
            }
        }; 
    }

    public LEDPattern presetPulsing(int hue, int sat, int speed){
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                pulsing(hue, sat, speed);
                return buffer;
            }
        };
    }

    /**
     * 3 rolling waves
     */
    public LEDPattern presetRollingWaves(int hue, int sat, int speed){
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                solid(hue, 255, kWaveThreshold);
                wave(hue, sat, 0, speed, kWaveLength, kWaveThreshold);
                wave(hue, sat, length/3, speed, kWaveLength, kWaveThreshold);
                wave(hue, sat, length/3*2, speed, kWaveLength, kWaveThreshold);
                return buffer;
            }
        };
    }

    public LEDPattern presetAutomaticRollingWaves(int speed){
        boolean blueTeam = DriverStation.getInstance().getAlliance() == Alliance.Blue;
        int hue = blueTeam ? kBlueHue : kRedHue;
        return presetRollingWaves(hue, 255, speed);
    }

    public LEDPattern presetRockingWave(int hueRange, int hueInitial){
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                for (int i = 0; i < length; i++) {
                    final int currHue = (int) (((Math.sin(workingHue / 180 * Math.PI) + 1) * (hueRange / 2.0)
                    + (i * hueRange / length)) % hueRange);
                    setHSV(i, currHue + hueInitial, 255, 255);
                }
                workingHue = (int) (Timer.getFPGATimestamp() * 80);
                
                workingHue %= 360;
                return buffer;
            }
        };
    }

    public LEDPattern presetTide(int hue, int minSat){
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                for (int i = 0; i < length; i++) {
                    int currSat = (int) (workingSat + (i * (255 - minSat) / length)) % 256;
                    setHSV(i, hue, currSat + minSat, 255);
                }
                workingSat = (int) (Timer.getFPGATimestamp() * 80) % 256;
                return buffer;
            }
        };
    }

    public LEDPattern presetFlashing(int hue, int sat, int val, int speed){
        return new LEDPattern(this){
            @Override
            public AddressableLEDBuffer draw() {
                int time = (int)(Timer.getFPGATimestamp()*speed);
                for(int i=0;i<length;i++){
                    if(time%2==0){
                        setHSV(i, hue, sat, val);
                    }
                    else{
                        setHSV(i, 0, 0, 0);
                    }
                }
                return buffer;
            }
        };
    }
}
