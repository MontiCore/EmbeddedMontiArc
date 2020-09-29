/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

import de.rwth.montisim.commons.utils.Vec2;

public class Path {
    public final double trajectoryX[];
    public final double trajectoryY[];
    public Path(int length){
        trajectoryX = new double[length];
        trajectoryY = new double[length];
    }
    protected Path() {
        trajectoryX = null;
        trajectoryY = null;
    }
    public int getLength(){
        return trajectoryX.length;
    }
    public void set(int index, double x, double y){
        trajectoryX[index] = x;
        trajectoryY[index] = y;
    }

    public void get(int index, Vec2 target){
        target.x = trajectoryX[index];
        target.y = trajectoryY[index];
    }
}