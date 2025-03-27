package org.wildstang.sample.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.sample.auto.Programs.BlueLeftThreeCoralV2;
import org.wildstang.sample.auto.Programs.LeftFourCoral;
import org.wildstang.sample.auto.Programs.BlueLeftFiveCoral;
import org.wildstang.sample.auto.Programs.TheDrakeBlue;
import org.wildstang.sample.auto.Programs.TheDrakeRed;
import org.wildstang.sample.auto.Programs.OldAutos.BlueRightThreeCoralV1;
import org.wildstang.sample.auto.Programs.OldAutos.Jubilee;
import org.wildstang.sample.auto.Programs.OldAutos.Line;
import org.wildstang.sample.auto.Programs.OldAutos.LineMix;
import org.wildstang.sample.auto.Programs.OldAutos.LinePose;
import org.wildstang.sample.auto.Programs.OldAutos.RedLeftThreeCoralV1;
import org.wildstang.sample.auto.Programs.OldAutos.RedRightThreeCoralV1;
import org.wildstang.sample.auto.Programs.OldAutos.TestProgram;


/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    //SAMPLE_PROGRAM("Sample", SampleAutoProgram.class),
    // Line("Line", Line.class),
    // LineMix("LineMix", LineMix.class),
    // LinePose("LinePose", LinePose.class),
    //TEST_PROGRAM("Test Program", TestProgram.class),
    //Jubilee("Jubilee", Jubilee.class),
    BLUE_LEFT_THREE_CORAL("Blue Left Three Coral V1", BlueLeftThreeCoralV2.class),
    BLUE_RIGHT_THREE_CORAL("Blue Right Three Coral V1", BlueRightThreeCoralV1.class),
    RED_LEFT_THREE_CORAL("Red Left Three Coral V1", RedLeftThreeCoralV1.class),
    RED_RIGHT_THREE_CORAL("Red Right Three Coral V1", RedRightThreeCoralV1.class),
    BLUE_LEFT_FIVE_CORAL("Blue Left Five Coral", BlueLeftFiveCoral.class),
    BLUE_LEFT_FOUR_CORAL("Blue Left Four Coral V1", LeftFourCoral.class),
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