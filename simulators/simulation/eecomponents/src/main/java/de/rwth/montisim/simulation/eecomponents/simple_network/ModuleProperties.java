/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import de.rwth.montisim.commons.utils.BuildContext;

public interface ModuleProperties {
    String getName();

    SimulatorModule build(BuildContext context);
}
