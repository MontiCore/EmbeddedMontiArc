/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagMinMaxTagSchema {

    protected static TagMinMaxTagSchema instance = null;

    protected TagMinMaxTagSchema() {
    }

    protected static TagMinMaxTagSchema getInstance() {
        if (instance == null) {
            instance = new TagMinMaxTagSchema();
        }
        return instance;
    }

    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator( new TagMinMaxSymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagMinMaxSymbol.KIND));
    }
}
