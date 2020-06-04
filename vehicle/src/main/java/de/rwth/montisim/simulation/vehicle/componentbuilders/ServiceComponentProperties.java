/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.componentbuilders;

import de.rwth.montisim.simulation.eesimulator.components.*;

public abstract class ServiceComponentProperties extends EEComponentProperties {

    public final String serviceType;
    public ServiceComponentProperties(String serviceType) {
        super(EEComponentType.SERVICE);
        this.serviceType = serviceType;
    }
    
}