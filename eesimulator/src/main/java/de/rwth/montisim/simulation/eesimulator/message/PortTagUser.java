/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Set;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;

// Use this interface for an EEComponent that needs to dynamically resolve
// messages from other components with specific tags.
public interface PortTagUser {
    // Return the tags the component is looking for
    Set<String> getUsedTags();

    // Gets called for every component port that contains a tag returned by getUsedTags().
    void processTag(String tag, PortInformation portInfo);
}
