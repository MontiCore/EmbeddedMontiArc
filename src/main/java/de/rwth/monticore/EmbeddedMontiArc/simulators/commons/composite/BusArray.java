/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.composite;

public class BusArray implements BusComponent {
    private BusComponent contents[];

    public BusArray(int size){
        this.contents = new BusComponent[size];
    }

    public void setComponent(int pos, BusComponent comp){
        contents[pos] = comp;
    }
    public BusComponent getComponent(int pos){
        return contents[pos];
    }

    public int getLength(){
        return contents.length;
    }

    BusComponent[] getContents(){
        return contents;
    }

    public ComponentType getType(){
        return ComponentType.ARRAY;
    }
}
