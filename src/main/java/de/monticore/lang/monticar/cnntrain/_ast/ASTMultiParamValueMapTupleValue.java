/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._ast;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 *
 */
public interface ASTMultiParamValueMapTupleValue extends ASTMultiParamValueMapTupleValueTOP {
    default ASTMultiParamValue getMultiParamValue() {
        return null;
    }
    default ASTStringValue getName() {
        return new ASTStringValue();
    }
}
