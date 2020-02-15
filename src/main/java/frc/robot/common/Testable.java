/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import frc.robot.util.Pair;

/**
 * Interface defining testable subsystems.
 * Testing a subsystem should return different states depending on the working status.
 */
public interface Testable {
    public enum Status{
        PASSED,
        WARNING,
        FAILED;
    }

    public class TestableResult{
        public final String name;
        public final Status status;

        public TestableResult(String name, Status status){
            this.name = name;
            this.status = status;
        }
    }

    /**
     * Tests the condition of this subsystem. Will return PASSED if not overriden.
     * @return {@link Status} of working condition.
     */
    public default TestableResult test(){return new TestableResult("Undefined", Status.PASSED);}
}
