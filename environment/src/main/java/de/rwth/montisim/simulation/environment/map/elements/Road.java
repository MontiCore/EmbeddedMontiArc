/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.map.elements;

import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.commons.utils.Vec3;

public class Road {
    public final String name;
    public final boolean oneWay;
    public final int lanes;
    public final List<Vec3> points = new ArrayList<>();
    public final boolean isArea;

    public Road(String name, boolean oneWay, int lanes, boolean isArea){
        this.isArea = isArea;
        this.name = name;
        this.oneWay = oneWay;
        this.lanes = lanes;
    }
}