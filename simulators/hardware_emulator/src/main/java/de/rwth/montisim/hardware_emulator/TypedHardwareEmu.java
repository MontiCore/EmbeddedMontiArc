/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator;

import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.simulation.TypedSimulation;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.*;

public abstract class TypedHardwareEmu {
    public static void registerTypedHardwareEmu() {
        TypedSimulation.registerTypedSimulation();
        Json.registerType(ComputerProperties.class);
        Json.registerType(Direct.class);
        Json.registerType(HardwareEmulator.class);
        Json.registerType(TCP.class);
        Json.registerType(MeasuredTime.class);
        Json.registerType(Realtime.class);
        Json.registerType(ConstantTime.class);
        Json.registerType(HardwareTimeModel.class);
    }
}
