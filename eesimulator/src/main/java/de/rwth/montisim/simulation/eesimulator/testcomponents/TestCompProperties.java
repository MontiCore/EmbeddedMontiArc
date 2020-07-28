/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

@Typed(TestCompProperties.TYPE)
public class TestCompProperties extends BusUserProperties {
	public static final String TYPE = "test_component";

    public TestCompProperties() {
    }

    protected TestCompProperties(String name) {
        this.name = name;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.TEST_COMPONENT;
    }

    @Override
    public String getType() {
        return TYPE;
    }

}