/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

/**
 * Tuple of two objects.
 */
public class Pair<A, B> {

    private final A a;
    private final B b;

    public Pair(A a, B b){
        this.a = a;
        this.b = b;
    }

    public A getPrimary(){return a;}
    public B getSecondary(){return b;}

    public static Pair<A, B> of(A a, B b) {
        return new Pair(a, b);
    }
}
