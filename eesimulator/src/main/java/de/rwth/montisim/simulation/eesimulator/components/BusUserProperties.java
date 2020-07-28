package de.rwth.montisim.simulation.eesimulator.components;

import java.util.Vector;

public abstract class BusUserProperties extends EEComponentProperties {
    public final Vector<String> buses = new Vector<>();

    public BusUserProperties(){
        buses.add("DefaultBus");
    }
}