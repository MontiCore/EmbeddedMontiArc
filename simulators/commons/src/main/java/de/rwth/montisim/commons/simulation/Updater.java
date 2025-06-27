/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.util.ArrayList;
import java.util.List;

/**
 * Stores references to different 'Updatable' objects.
 * 
 * applyUpdate() will update all registered objects.
 */
public class Updater {
    protected transient List<Updatable> updatables = new ArrayList<>();
    
    /** Register an Updatable. */
    public void addUpdatable(Updatable u){
        updatables.add(u);
    }

    /** Update all registered updatables. */
    public void applyUpdate(TimeUpdate newTime){
        for (Updatable u : updatables){
            u.update(newTime);
        }
    }
}