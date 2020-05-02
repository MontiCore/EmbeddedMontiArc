package de.rwth.montisim.simulation.simulator.visualization.plotter;

import java.util.Vector;

public class DataPoints {
    public final String name;
    public int nameWidth; // Used by the plotter

    Vector<Double> data = new Vector<>();
    public boolean discrete = false;
    public boolean showZero = true;
    private int index = 0;

    public DataPoints(String name){
        this.name = name;
        data.setSize(1);
    }

    public void add(double v){
        data.set(index, v);
    }

    public void nextTimeStep(){
        data.setSize(data.size()+1);
        index++;
    }
}