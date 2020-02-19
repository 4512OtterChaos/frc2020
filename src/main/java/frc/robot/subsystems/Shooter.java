/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable, Testable{

  @Log(methodName = "getAppliedOutput")
  private CANSparkMax shootMaster;
  private CANSparkMax shootSlave;
  private CANSparkMax shootWrist;

  private CANEncoder shootEncoder;
  
  
  public Shooter() {
    shootMaster = OCConfig.createMAX(5, ConfigType.SHOOTER);
    shootSlave = OCConfig.createMAX(6, ConfigType.SHOOTER);
    shootWrist = OCConfig.createMAX(7, ConfigType.SHOOTERWRIST);
  }

  @Override
  public void periodic() {
  }

  @Override
  public TestableResult test(){
    return new TestableResult("Shooter", Status.PASSED);
  }
}
