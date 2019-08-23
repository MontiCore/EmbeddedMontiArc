/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagDelayTagSchema {

    protected static TagDelayTagSchema instance = null;

    protected TagDelayTagSchema() {
    }

    protected static TagDelayTagSchema getInstance() {
        if (instance == null) {
            instance = new TagDelayTagSchema();
        }
        return instance;
    }

    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagDelaySymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagDelaySymbol.KIND));
    }
}
