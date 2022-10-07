/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import java.util.Vector;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(TestCompProperties.TYPE)
public class TestCompProperties extends EEComponentProperties {
    public static final String TYPE = "test_component";

    public Vector<PortInformation> ports = new Vector<>();

    public TestCompProperties() {
    }

    public TestCompProperties(String name) {
        this.name = name;
    }

    public TestCompProperties addPort(PortInformation inf) {
        ports.add(inf);
        return this;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.TEST_COMPONENT;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEComponent build(EESystem eesystem, BuildContext context) {
        return new TestEEComponent(this, eesystem);
    }

}