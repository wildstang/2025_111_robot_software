package org.wildstang.framework.io;

import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.wildstang.framework.CoreUtils;
import org.wildstang.framework.core.Inputs;
import org.wildstang.framework.io.inputs.Input;

/**
 * Contains and updates all Inputs.
 *
 * @author Steve
 */
public class InputManager {

    private static Logger s_log = Logger.getLogger(InputManager.class.getName());
    private static final String s_className = "InputManager";

    private HashMap<String, Input> m_inputs = new HashMap<>();
    private boolean s_initialised = false;

    /**
     * Initializes logger.
     */
    public void init() {
        s_log.entering(s_className, "init");

        if (!s_initialised) {
            s_initialised = true;
        }

        s_log.exiting(s_className, "init");
    }

    /**
     * Updates all managed inputs.
     */
    public void update() {
        if (s_log.isLoggable(Level.FINER)) {
            s_log.entering(s_className, "update");
        }

        // Iterate over all inputs and update each one
        for (Input in : m_inputs.values()) {
            if (in.isEnabled()) {
                if (s_log.isLoggable(Level.FINEST)) {
                    s_log.finest("Updating Input: " + in.getName());
                }

                // Update the input
                in.update();
            } else {
                if (s_log.isLoggable(Level.FINEST)) {
                    s_log.finest("Input " + in.getName() + " is not enabled. Not calling update.");
                }
            }
        }

        if (s_log.isLoggable(Level.FINER)) {
            s_log.exiting(s_className, "update");
        }
    }

    /**
     * Adds an Input to the manager.
     * @param p_input Input object to add.
     */
    public void addInput(Input p_input) {
        CoreUtils.checkNotNull(p_input, "p_input is null");

        // If this input has not already been added, add it
        // Even if it exists, we don't want to replace it as it may already have
        // state
        if (!m_inputs.containsKey(p_input.getName())) {
            m_inputs.put(p_input.getName(), p_input);
        }
    }

    /**
     * Removes an Input to the manager.
     * @param p_input Input object to remove.
     */
    public void removeInput(Input p_input) {
        CoreUtils.checkNotNull(p_input, "p_input is null");

        if (s_log.isLoggable(Level.WARNING)) {
            s_log.warning("Removing input " + p_input.getName());
        }

        m_inputs.remove(p_input.getName());
    }

    /**
     * Returns a named Input from the manager.
     * @param p_name Name of the Input to fetch.
     * @return Input belonging to the manager.
     */
    public Input getInput(String p_name) {
        CoreUtils.checkNotNull(p_name, "p_name is null");
        Input result = null;

        if (!m_inputs.containsKey(p_name)) {
            throw new NoSuchElementException("No input with name '" + p_name + "' in InputManager");
        }
        result = m_inputs.get(p_name);

        return result;
    }

    /**
     * Returns an Input with a matching name from the manager.
     * @param p_name Input to fetch from the manager.
     * @return Input belonging to the manager.
     */
    public Input getInput(Inputs p_input) {
        return getInput(p_input.getName());
    }

    /**
     * Determines if the manager has an Input with a given name.
     * @param p_name Name of an Input to search for.
     * @return True if an Input exists with p_name.
     */
    public boolean contains(String p_name) {
        CoreUtils.checkNotNull(p_name, "p_name is null");
        return m_inputs.containsKey(p_name);
    }

    /**
     * Returns the total size of all managed inputs.
     * @return Size of all managed inputs.
     */
    public int size() {
        return m_inputs.size();
    }

    /**
     * Removes all Inputs from the manager.
     */
    public void removeAll() {
        m_inputs.clear();
    }

    /**
     * Produces a HashMap of all managed Inputs.
     * @return HashMap of all managed Inputs mapped to their names.
     */
    public HashMap<String, Input> getHashMap() {
        return new HashMap<>(m_inputs);
    }
}
