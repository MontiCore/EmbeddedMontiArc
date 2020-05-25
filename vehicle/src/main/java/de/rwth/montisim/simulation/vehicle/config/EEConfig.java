/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.config;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;

public class EEConfig {
    public final Vector<EEComponentProperties> components = new Vector<>();

    public Optional<EEComponentProperties> getProperties(String componentName){
        return components.stream().filter(x -> x.name.equals(componentName)).findFirst();
    }

    public void addComponent(EEComponentProperties properties){
        components.add(properties);
    }
}