/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.componentbuilders;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public abstract class ComputerComponentProperties extends EEComponentProperties {

    public final String computerType;
    public ComputerComponentProperties(String serviceType) {
        super(EEComponentType.COMPUTER);
        this.computerType = serviceType;
    }
    
}