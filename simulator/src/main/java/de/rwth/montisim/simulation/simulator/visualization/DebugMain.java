package de.rwth.montisim.simulation.simulator.visualization;

import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;

public class DebugMain {

    public static void main(String args[]) throws EESetupException {
        //new DebugVisualizer(args);
        new PhysicsDebug(args);
    }
}