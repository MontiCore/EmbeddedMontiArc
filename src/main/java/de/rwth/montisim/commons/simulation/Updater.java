/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.util.ArrayList;
import java.util.List;

public class Updater {
    protected List<Updatable> updatables = new ArrayList<>();
    
    public void addUpdatable(Updatable u){
        updatables.add(u);
    }

    public void applyUpdate(TimeUpdate newTime){
        for (Updatable u : updatables){
            u.update(newTime);
        }
    }
}