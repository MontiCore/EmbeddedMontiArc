/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.util.ArrayList;
import java.util.List;

/**
 * Stores references to different 'Destroyable' objects.
 * 
 * applyDestroy() will destroy all registered objects.
 */
public class Destroyer {
    protected transient List<Destroyable> destroyables = new ArrayList<>();
    
    /** Register an Updatable. */
    public void addDestroyable(Destroyable d){
        destroyables.add(d);
    }

    /** Update all registered updatables. */
    public void applyDestroy(){
        for (Destroyable d : destroyables){
            d.destroy();
        }
    }
}