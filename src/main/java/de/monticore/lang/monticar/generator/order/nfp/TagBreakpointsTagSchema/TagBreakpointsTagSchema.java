/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema;

import de.monticore.CommonModelingLanguage;
import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class TagBreakpointsTagSchema {

    protected static TagBreakpointsTagSchema instance = null;

    protected TagBreakpointsTagSchema() {
    }

    protected static TagBreakpointsTagSchema getInstance() {
        if (instance == null) {
            instance = new TagBreakpointsTagSchema();
        }
        return instance;
    }


    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagBreakpointsSymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagBreakpointsSymbol.KIND));
    }
}
