/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.someip.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.*;

public class SomeIPTagHelper {
    private SomeIPTagHelper() {
    }

    public static Map<EMAPortSymbol, SomeIPConnectionSymbol> resolveTags(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) {
        Map<EMAPortSymbol, SomeIPConnectionSymbol> someIPConnectionSymbols = new HashMap<>();
            componentInstanceSymbol.getPortInstanceList().forEach(p -> {
                Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, SomeIPConnectionSymbol.KIND);
                if (tmpTags.size() == 1) {
                    someIPConnectionSymbols.put(p, (SomeIPConnectionSymbol) tmpTags.iterator().next());
                }
            });
            componentInstanceSymbol.getSubComponents().forEach(sc -> someIPConnectionSymbols.putAll(SomeIPTagHelper.resolveTags(taggingResolver, sc)));
        return someIPConnectionSymbols;
    }
}