/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.vcg;

import java.io.IOException;
import java.time.Duration;

import de.rwth.montisim.commons.dynamicinterface.ProgramInterface;
import de.rwth.montisim.hardware_emulator.vcg.VCGProperties.TimeMode;

public interface CommunicationInterface {
    ProgramInterface init(TimeMode timeMode, int ref_id) throws Exception;
    boolean isAlive();
    Duration measuredCycle(Object portData[], double deltaSec) throws IOException ;
}
