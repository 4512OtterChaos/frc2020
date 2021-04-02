/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.TreeMap;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
* 
*/
public class OCLEDManager {
    
    private static OCLEDManager instance;
    
    public static OCLEDManager getInstance(){
        return instance;
    }
    
    public enum Configuration{
        NONE, // normal
        COPYSPLIT, // normal, but copy first half onto second 
        MIRRORSPLIT // start patterns from both ends and meet in the middle
    }
    
    private TreeMap<PatternCard, LEDPattern> patternList = new TreeMap<PatternCard, LEDPattern>(new Comparator<PatternCard>(){
        public int compare(PatternCard a, PatternCard b){
            return Integer.compare(a.getPriority(), b.getPriority());
        };
    });
    
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private Configuration config;
    
    /**
    * Create an LED manager for a new LED strip.
    * @param ledPort PWM port of LED strip
    * @param length Pixel length of strip
    */
    public OCLEDManager(int ledPort, int length, Configuration config){
        instance = this;
        this.config = config;
        led = new AddressableLED(ledPort);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        led.setData(buffer);
        led.start();
        
        addPattern(new PatternCard(()->false, -1), new LEDPattern(this).presetAutomaticRollingWaves(20)); // if no other patterns, we display this pattern by default
    }
    
    
    /**
    * Set whether patterns are in a split-configuration(start patterns from both ends and meet in the middle).
    * Useful on strips visible from two sides of the robot(e.g. two sides of an elevator).
    */
    /*
    public void configure(Configuration config){
        this.config = config;
    }
    */
    
    /**
    * @return Whether the LED strip is in split-configuration(start patterns from both ends and meet in the middle).
    */
    public Configuration getConfig(){
        return config;
    }
    
    /**
    * @return Total pixel length of the LED strip
    */
    public int getTotalStripLength(){
        return buffer.getLength();
    }
    /**
    * @return "Configured" length of LED strip(e.g. half total length when in split-configuration)
    */
    public int getConfiguredStripLength(){
        // halve the strip depending on config
        return 
        (config == Configuration.MIRRORSPLIT ||
        config == Configuration.COPYSPLIT) ? buffer.getLength()/2 : buffer.getLength();
    }
    
    public void addPattern(PatternCard card, LEDPattern pattern){
        if(patternList.containsValue(pattern)){ // refresh current patterns
            for(Entry<PatternCard, LEDPattern> entry : patternList.entrySet()){
                if(entry.getValue().equals(pattern) && entry.getKey().getDuration() > 0)
                entry.getKey().extend(card.getDuration());
            }
        }
        else patternList.put(card, pattern); // add new patterns
    }
    
    public void periodic(){
        List<PatternCard> toRemove = new ArrayList<PatternCard>();
        patternList.forEach((card, pattern) -> {
            if(card.isFinished()) toRemove.add(card);
        });
        toRemove.forEach((card)->{
            patternList.remove(card);
        });
        buffer = patternList.lastEntry().getValue().draw();
        led.setData(buffer);
    }
    
    //-----
    
    
    /**
    * For giving {@link LEDPattern}s a priority and duration.
    * Call end() to manually end a pattern.
    */
    public class PatternCard {
        private final double timeStart;
        private double duration;
        private final BooleanSupplier interrupt; // if interrupted, end pattern
        private final int priority;
        private boolean finished = false; // end pattern based on this boolean
        
        /**
        * Create a new PatternCard for an associated {@link LEDPattern} that lasts <b>duration</b> seconds and
        * has a display <b>priority</b> over other, lower priorities(create an indefinite PatternCard with priority -1 for the base/default LEDPattern).
        * @param duration
        * @param priority
        */
        public PatternCard(double duration, int priority){
            timeStart = Timer.getFPGATimestamp();
            this.duration = duration;
            this.interrupt = ()->false;
            this.priority = priority;
        }
        /**
        * Create a new PatternCard for an associated {@link LEDPattern} that lasts until <b>interrupt</b> is true and
        * has a display <b>priority</b> over other, lower priorities(create an indefinite PatternCard with priority -1 for the base/default LEDPattern).
        * @param duration
        * @param priority
        */
        public PatternCard(BooleanSupplier interrupt, int priority){
            timeStart = Timer.getFPGATimestamp();
            this.duration = -1;
            this.interrupt = interrupt;
            this.priority = priority;
        }
        /**
        * Create a new PatternCard for an associated {@link LEDPattern} that lasts <b>duration</b> seconds or until <b>interrupt</b> is true and
        * has a display <b>priority</b> over other, lower priorities(create an indefinite PatternCard with priority -1 for the base/default LEDPattern).
        * @param duration
        * @param priority
        */
        public PatternCard(double duration, BooleanSupplier interrupt, int priority){
            timeStart = Timer.getFPGATimestamp();
            this.duration = duration;
            this.interrupt = interrupt;
            this.priority = priority;
        }
        
        public int getPriority(){
            return priority;
        }
        
        public double getDuration(){
            return duration;
        }
        
        public void extend(double time){
            duration += time;
        }
        
        /**
        * Manually end this pattern.
        */
        public void end(){
            finished = true;
        }
        
        /**
        * Check if this pattern should be ended
        */
        public boolean isFinished(){
            if(interrupt.getAsBoolean()) end();
            if(duration > 0 && Timer.getFPGATimestamp() > timeStart + duration) end();
            
            return finished;
        }
    }
}