/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.plotter;

import java.awt.Color;
import java.util.*;

public class DataPoints {
    public static final int BLOCK_SIZE = 1024;
    public final String name;
    public final String unit;
    public int nameWidth; // Used by the plotter

    Vector<double[]> data = new Vector<>();
    public boolean discrete = false;
    public boolean showZero = true;
    private int index = 0;
    private int arrayIndex = 0;
    Optional<Color> color = Optional.empty();

    public DataPoints(String name, String unit, Color color) {
        this.name = name;
        this.unit = unit;
        data.add(new double[BLOCK_SIZE]);
        this.color = Optional.of(color);
    }

    public DataPoints(String name, String unit) {
        this.name = name;
        this.unit = unit;
        data.add(new double[BLOCK_SIZE]);
    }

    public void setTickData(double v) {
        data.elementAt(arrayIndex)[index] = v;
    }

    public void nextTimeStep() {
        index++;
        if (index == BLOCK_SIZE) {
            index = 0;
            data.add(new double[BLOCK_SIZE]);
            arrayIndex++;
        }
    }
}