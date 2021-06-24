/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.time.Duration;

import de.rwth.montisim.commons.dynamicinterface.ProgramInterface;
import de.rwth.montisim.commons.simulation.Destroyable;

public interface ComputerBackend extends Destroyable {
    ProgramInterface getInterface();
    Duration measuredCycle(Object portData[], double deltaSec) throws Exception;
}
