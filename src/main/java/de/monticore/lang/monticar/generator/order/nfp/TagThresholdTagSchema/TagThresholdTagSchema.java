/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagThresholdTagSchema {

    protected static TagThresholdTagSchema instance = null;

    protected TagThresholdTagSchema() {
    }

    protected static TagThresholdTagSchema getInstance() {
        if (instance == null) {
            instance = new TagThresholdTagSchema();
        }
        return instance;
    }

    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagThresholdSymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagThresholdSymbol.KIND));
    }
}
