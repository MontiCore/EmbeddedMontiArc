/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema;

import de.monticore.lang.monticar.generator.order.ExecutionOrder;
import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagExecutionOrderSymbol extends TagSymbol {
  public static final TagExecutionOrderSymbolKind KIND = TagExecutionOrderSymbolKind.INSTANCE;

  public TagExecutionOrderSymbol(ExecutionOrder executionOrder) {
    super(KIND, executionOrder);
  }

  @Override
  public String toString() {
    return String.format("TagExecutionOrder = %s",
    getExecutionOrder());
  }

  public ExecutionOrder getExecutionOrder() {
    return getValue(0);
  }

  public static class TagExecutionOrderSymbolKind extends TagKind {

    public static final TagExecutionOrderSymbolKind INSTANCE = new TagExecutionOrderSymbolKind();

    protected TagExecutionOrderSymbolKind() {}
  }
}


