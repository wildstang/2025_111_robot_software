package org.wildstang.hardware.roborio;

import java.util.logging.Level;
import java.util.logging.Logger;

import org.wildstang.framework.core.Outputs;
import org.wildstang.framework.hardware.OutputConfig;
import org.wildstang.framework.io.outputs.Output;
import org.wildstang.hardware.roborio.outputs.WsDigitalOutput;
import org.wildstang.hardware.roborio.outputs.WsDoubleSolenoid;
import org.wildstang.hardware.roborio.outputs.WsI2COutput;
import org.wildstang.hardware.roborio.outputs.WsPhoenix;
import org.wildstang.hardware.roborio.outputs.WsRelay;
import org.wildstang.hardware.roborio.outputs.WsServo;
import org.wildstang.hardware.roborio.outputs.WsSolenoid;
import org.wildstang.hardware.roborio.outputs.WsTalon;
import org.wildstang.hardware.roborio.outputs.WsVictor;
import org.wildstang.hardware.roborio.outputs.WsRemoteAnalogOutput;
import org.wildstang.hardware.roborio.outputs.WsRemoteDigitalOutput;
import org.wildstang.hardware.roborio.outputs.config.WsDigitalOutputConfig;
import org.wildstang.hardware.roborio.outputs.config.WsDoubleSolenoidConfig;
import org.wildstang.hardware.roborio.outputs.config.WsI2COutputConfig;
import org.wildstang.hardware.roborio.outputs.config.WsPhoenixConfig;
import org.wildstang.hardware.roborio.outputs.config.WsRelayConfig;
import org.wildstang.hardware.roborio.outputs.config.WsServoConfig;
import org.wildstang.hardware.roborio.outputs.config.WsSolenoidConfig;
import org.wildstang.hardware.roborio.outputs.config.WsTalonConfig;
import org.wildstang.hardware.roborio.outputs.config.WsVictorConfig;
import org.wildstang.hardware.roborio.outputs.config.WsRemoteAnalogOutputConfig;
import org.wildstang.hardware.roborio.outputs.config.WsRemoteDigitalOutputConfig;

/**
 * Builds outputs from WsOutputs enumerations.
 */
public class RoboRIOOutputFactory {

    private static Logger s_log = Logger.getLogger(RoboRIOOutputFactory.class.getName());
    private static final String s_className = "RoboRIOOutputFactory";
    private boolean s_initialised = false;

    /**
     * Empty constructor override WPILib InputFactory constructor.
     */
    public RoboRIOOutputFactory() {}

    /**
     * Prepares logger.
     */
    public void init() {
        if (s_log.isLoggable(Level.FINER)) {
            s_log.entering(s_className, "init");
        }

        if (!s_initialised) {
            s_initialised = true;
        }

        if (s_log.isLoggable(Level.FINER)) {
            s_log.exiting(s_className, "init");
        }
    }

    /**
     * Creates an Output from an enumeration of WsOutputs.
     * @param p_output An enumeration of WsOutputs.
     * @return A constructed Output.
     */
    public Output createOutput(Outputs p_output) {
        if (s_log.isLoggable(Level.FINER)) {
            s_log.entering(s_className, "createDigitalInput");
        }

        Output out = null;
        OutputConfig config = p_output.getConfig();

        if (s_log.isLoggable(Level.FINE)) {
            s_log.fine("Creating digital output: Name = " + p_output.getName());
        }

        if (config instanceof WsServoConfig) {
            WsServoConfig c = (WsServoConfig) config;
            out = new WsServo(p_output.getName(), c.getChannel(), c.getDefault());
        }
        else if (config instanceof WsRelayConfig) {
            WsRelayConfig c = (WsRelayConfig) config;
            out = new WsRelay(p_output.getName(), c.getChannel());
        }
        else if (config instanceof WsPhoenixConfig) {
            WsPhoenixConfig c = (WsPhoenixConfig) config;
            out = new WsPhoenix(p_output.getName(), c.getChannel(), c.getDefault(),
                                c.isTalon(), c.isInverted());
        }
        else if (config instanceof WsVictorConfig) {
            WsVictorConfig c = (WsVictorConfig) config;
            out = new WsVictor(p_output.getName(), c.getChannel(), c.getDefault());
        }
        else if (config instanceof WsTalonConfig) {
            WsTalonConfig c = (WsTalonConfig) config;
            out = new WsTalon(p_output.getName(), c.getChannel(), c.getDefault());
        }
        else if (config instanceof WsSolenoidConfig) {
            WsSolenoidConfig c = (WsSolenoidConfig) config;
            out = new WsSolenoid(p_output.getName(), c.getModule(), c.getChannel(), c.getDefault());
        }
        else if (config instanceof WsDoubleSolenoidConfig) {
            WsDoubleSolenoidConfig c = (WsDoubleSolenoidConfig) config;
            out = new WsDoubleSolenoid(p_output.getName(), c.getModule(), c.getChannel1(),
                                        c.getChannel2(), c.getDefault());
        }
        else if (config instanceof WsI2COutputConfig) {
            WsI2COutputConfig c = (WsI2COutputConfig) config;
            out = new WsI2COutput(p_output.getName(), c.getPort(), c.getAddress());
        }
        else if (config instanceof WsRemoteDigitalOutputConfig) {
            WsRemoteDigitalOutputConfig c = (WsRemoteDigitalOutputConfig) config;
            out = new WsRemoteDigitalOutput(p_output.getName(), c.getTableName(),c.getDefault());
        }
        else if (config instanceof WsRemoteAnalogOutputConfig) {
            WsRemoteAnalogOutputConfig c = (WsRemoteAnalogOutputConfig) config;
            out = new WsRemoteAnalogOutput(p_output.getName(), c.getTableName(), c.getDefault());
        }
        else if (config instanceof WsDigitalOutputConfig) {
            WsDigitalOutputConfig c = (WsDigitalOutputConfig) config;
            out = new WsDigitalOutput(p_output.getName(), c.getChannel(), c.getDefault());
        }

        if (s_log.isLoggable(Level.FINER)) {
            s_log.exiting(s_className, "createDigitalInput");
        }

        return out;
    }

}