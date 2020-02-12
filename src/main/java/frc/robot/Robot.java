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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.Testable;
import frc.robot.common.Testable.Status;
import frc.robot.util.Pair;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private RobotContainer container;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    container = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
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
    List<Pair<String,Status>> results = new ArrayList<Pair<String,Status>>();
    for(Testable sub:container.getTestableSubsystems()){
      Pair<String,Status> result = sub.test();
      results.add(result);
    }
    for(Pair<String,Status> result:results){
      Status status = result.getSecondary();

      //LED blank
      Timer.delay(0.25);
      System.out.println(result.getPrimary()+": "+status.toString());
      switch(status){
        case FAILED:
        //LED failed
        Timer.delay(0.75);
        break;
        case WARNING:
        //LED warning
        Timer.delay(0.5);
        break;
        case PASSED:
        //LED passed
        Timer.delay(0.25);
        break;
      }
    }
  }

  @Override
  public void testPeriodic() {

  }
}
