/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import static frc.robot.common.Constants.*;

import frc.robot.common.OCLEDManager;
import frc.robot.common.Testable;
import frc.robot.common.Testable.Status;
import frc.robot.common.Testable.TestableResult;

public class Robot extends TimedRobot {
    
    private RobotContainer container;
    
    private Command autoCommand;
    
    private Timer disableTimer = new Timer();
    
    public Robot(){
        super(kRobotDelta);
    }

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        
        container = new RobotContainer();
        
        addPeriodic(container::log, 0.04, 0.02);
    }
    
    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        container.periodic();
        //CommandScheduler.getInstance().onCommandFinish((command)->SmartDashboard.putString("Last Command Ended", command.getName()));
    }
    
    @Override
    public void disabledInit() {
        disableTimer.reset();
        disableTimer.start();
        
        container.stop();
        container.disable();
    }
    
    @Override
    public void disabledPeriodic() {
        if(disableTimer.get()>2){
            disableTimer.stop();
            disableTimer.reset();
            
            container.setAllBrake(false);
        }
    }
    
    @Override
    public void autonomousInit() {
        container.setAllBrake(true);
        container.init(true);
        
        autoCommand = container.getAutonomousCommand();
        
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }
    
    @Override
    public void autonomousPeriodic() {
    }
    
    @Override
    public void teleopInit() {
        container.setAllBrake(true);
        container.init(false);
        
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {
    }
    
    @Override
    public void testInit() {
        //LED stuff to show testing
        Timer.delay(1);
        
        CommandScheduler.getInstance().cancelAll();
        List<TestableResult> results = new ArrayList<TestableResult>();
        for(Testable system:container.getTestableSystems()){
            TestableResult result = system.test();
            results.add(result);
        }
        /*
        for(TestableResult result:results){
            Status status = result.status;
            
            Pattern prevPattern = OCLedManager.getPattern();
            OCLedManager.setPattern(Pattern.YellowDash);
            Timer.delay(0.5);
            System.out.println(result.name+": "+status.toString());
            switch(status){
                case FAILED:
                OCLedManager.setPattern(Pattern.RedPulsing);
                Timer.delay(1.5);
                break;
                case WARNING:
                OCLedManager.setPattern(Pattern.YellowPulsing);
                Timer.delay(1);
                break;
                case PASSED:
                OCLedManager.setPattern(Pattern.GreenPulsing);
                Timer.delay(0.5);
                break;
            }
            Timer.delay(1);
            OCLedManager.setPattern(prevPattern);
        }
        */
    }
    
    @Override
    public void testPeriodic() {
        
    }
}
