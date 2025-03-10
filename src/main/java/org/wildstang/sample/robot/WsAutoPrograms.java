package org.wildstang.sample.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.sample.auto.Programs.Jubilee;
import org.wildstang.sample.auto.Programs.Line;
import org.wildstang.sample.auto.Programs.RedLeftThreeCoralV1;
import org.wildstang.sample.auto.Programs.RedRightThreeCoralV1;
import org.wildstang.sample.auto.Programs.TestProgram;
import org.wildstang.sample.auto.Programs.TheDrakeBlue;
import org.wildstang.sample.auto.Programs.TheDrakeRed;
import org.wildstang.sample.auto.Programs.BlueLeftThreeCoralV1;
import org.wildstang.sample.auto.Programs.BlueRightThreeCoralV1;


/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    //SAMPLE_PROGRAM("Sample", SampleAutoProgram.class),
    //Line("Line", Line.class),
    //TEST_PROGRAM("Test Program", TestProgram.class),
    //Jubilee("Jubilee", Jubilee.class),
    BLUE_LEFT_THREE_CORAL("Blue Left Three Coral V1", BlueLeftThreeCoralV1.class),
    BLUE_RIGHT_THREE_CORAL("Blue Right Three Coral V1", BlueRightThreeCoralV1.class),
    RED_LEFT_THREE_CORAL("Red Left Three Coral V1", RedLeftThreeCoralV1.class),
    RED_RIGHT_THREE_CORAL("Red Right Three Coral V1", RedRightThreeCoralV1.class),
    CENTER_BLUE("Center Blue", TheDrakeBlue.class),
    CENTER_RED("Center Red", TheDrakeRed.class),

    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> programClass;

    /**
     * Initialize name and AutoProgram map.
     * @param name Name, must match that in class to prevent errors.
     * @param programClass Class containing AutoProgram
     */
    WsAutoPrograms(String name, Class<?> programClass) {
        this.name = name;
        this.programClass = programClass;
    }

    /**
     * Returns the name mapped to the AutoProgram.
     * @return Name mapped to the AutoProgram.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns AutoProgram's class.
     * @return AutoProgram's class.
     */
    @Override
    public Class<?> getProgramClass() {
        return programClass;
    }
}