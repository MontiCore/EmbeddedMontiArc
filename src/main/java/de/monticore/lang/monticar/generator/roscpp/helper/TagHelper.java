package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class TagHelper {
    private static Set<String> allreadyResolved = new HashSet<>();

    private TagHelper() {
    }

    public static Map<PortSymbol, RosConnectionSymbol> resolveTags(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        Map<PortSymbol, RosConnectionSymbol> rosConnectionSymbols = new HashMap<>();
        if (!allreadyResolved.contains(componentInstanceSymbol.getFullName())) {
            componentInstanceSymbol.getPorts().forEach(p -> {
                Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, RosConnectionSymbol.KIND);
                if (tmpTags.size() == 1) {
                    rosConnectionSymbols.put(p, (RosConnectionSymbol) tmpTags.iterator().next());
                }
            });
            componentInstanceSymbol.getSubComponents().forEach(sc -> rosConnectionSymbols.putAll(TagHelper.resolveTags(taggingResolver, sc)));
            allreadyResolved.add(componentInstanceSymbol.getFullName());
        }
        return rosConnectionSymbols;
    }

    public static List<File> generate(GeneratorRosCpp generatorRosCpp, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        Map<PortSymbol, RosConnectionSymbol> allRosTags = resolveTags(taggingResolver, componentInstanceSymbol);
        checkTags(allRosTags);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }

    private static void checkTags(Map<PortSymbol, RosConnectionSymbol> rosTags) {
        rosTags.forEach((port, tag) -> {
            if (!tag.getTopicName().isPresent())
                Log.error("Every topicName needs to be set but " + port.getFullName() + " has a RosConnectionSymbol without one!");

            if (!tag.getTopicType().isPresent())
                Log.error("Every topicType needs to be set but " + port.getFullName() + " has a RosConnectionSymbol without one!");

            if (!tag.getMsgField().isPresent())
                Log.error("Every msgField needs to be set but " + port.getFullName() + " has a RosConnectionSymbol without one!");
        });

    }

    public static void reset() {
        allreadyResolved.clear();
    }
}
