/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.JsonEntry;

public enum LTLOperator {
    @JsonEntry("always")
    ALWAYS,
    @JsonEntry("never")
    NEVER, 
    @JsonEntry("eventually")
    EVENTUALLY, 
    @JsonEntry("until")
    UNTIL
}

