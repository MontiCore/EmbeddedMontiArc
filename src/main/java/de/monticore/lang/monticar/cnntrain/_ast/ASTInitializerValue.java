/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._ast;

import java.util.List;

public interface ASTInitializerValue extends ASTInitializerValueTOP {

    String getName();

    List<? extends ASTEntry> getParamsList();
}
