/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._ast;

import java.util.List;

public interface ASTLossValue extends ASTLossValueTOP{
    String getName();

    List<? extends ASTEntry> getParamsList();
}
