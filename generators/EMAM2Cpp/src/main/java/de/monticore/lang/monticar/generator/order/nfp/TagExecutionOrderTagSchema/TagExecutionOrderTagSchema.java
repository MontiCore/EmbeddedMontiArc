/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagExecutionOrderTagSchema {

    protected static TagExecutionOrderTagSchema instance = null;

    protected TagExecutionOrderTagSchema() {
    }

    protected static TagExecutionOrderTagSchema getInstance() {
        if (instance == null) {
            instance = new TagExecutionOrderTagSchema();
        }
        return instance;
    }

    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagExecutionOrderSymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagExecutionOrderSymbol.KIND));
    }
}
