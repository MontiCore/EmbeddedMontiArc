/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.world.elements;

import java.util.Vector;

import de.rwth.montisim.commons.utils.Vec3;

public class Building {
    public final String name;
    public final String type;
    public final double height;
    public final int levels;
    public final Vector<Vec3> boundary = new Vector<>();

    public Building(String name, String type, double height, int levels){
        this.name = name;
        this.type = type;
        this.height = height;
        this.levels = levels;
    }
}