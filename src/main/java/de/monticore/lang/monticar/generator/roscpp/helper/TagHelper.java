package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class TagHelper {
    public static void resolveTags(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        componentInstanceSymbol.getPorts().forEach(p -> taggingResolver.getTags(p, RosConnectionSymbol.KIND));
        componentInstanceSymbol.getSubComponents().forEach(sc -> TagHelper.resolveTags(taggingResolver, sc));
    }

    public static List<File> generate(GeneratorRosCpp generatorRosCpp, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        resolveTags(taggingResolver, componentInstanceSymbol);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }
}
