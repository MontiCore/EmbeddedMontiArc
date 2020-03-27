/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.map.elements;

import java.util.Vector;

import de.rwth.montisim.commons.utils.Vec2;

public class Building {
    public final String name;
    public final String type;
    public final double height;
    public final int levels;
    public final Vector<Vec2> boundary = new Vector<>();

    public Building(String name, String type, double height, int levels){
        this.name = name;
        this.type = type;
        this.height = height;
        this.levels = levels;
    }
}