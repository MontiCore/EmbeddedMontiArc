/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.componentbuilders;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public abstract class ServiceComponentProperties extends EEComponentProperties {

    public final String serviceType;
    public ServiceComponentProperties(String serviceType) {
        super(EEComponentType.SERVICE);
        this.serviceType = serviceType;
    }
    
}