/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public class TestCompProperties extends EEComponentProperties {

    protected TestCompProperties(String name) {
        super(EEComponentType.TEST_COMPONENT);
        this.name = name;
    }
    
}