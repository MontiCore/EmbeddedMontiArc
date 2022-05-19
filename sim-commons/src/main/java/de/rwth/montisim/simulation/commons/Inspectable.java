/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons;

import java.util.List;

public interface Inspectable {
    String getType(); // A general type for the inspectable object (ex: "autopilot")

    String getName(); // The name of the inspected element

    List<String> getEntries(); // Returns a list of debug values for the object.
}
