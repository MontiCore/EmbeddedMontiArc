/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.JsonEntry;

public enum  Comparator {
    @JsonEntry("<")
    LESS,
    @JsonEntry("<=")
    LESS_EQUAL, 
    @JsonEntry(">")
    GREATER, 
    @JsonEntry(">=")
    GREATER_EQUAL, 
    @JsonEntry("==")
    EQUAL
}
