package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class TagHelper {
    private TagHelper() {
    }

    public static Map<PortSymbol, RosConnectionSymbol> resolveTags(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        Map<PortSymbol, RosConnectionSymbol> rosConnectionSymbols = new HashMap<>();
            componentInstanceSymbol.getPortsList().forEach(p -> {
                Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, RosConnectionSymbol.KIND);
                if (tmpTags.size() == 1) {
                    rosConnectionSymbols.put(p, (RosConnectionSymbol) tmpTags.iterator().next());
                }
            });
            componentInstanceSymbol.getSubComponents().forEach(sc -> rosConnectionSymbols.putAll(TagHelper.resolveTags(taggingResolver, sc)));
        return rosConnectionSymbols;
    }

    public static List<File> resolveAndGenerate(GeneratorRosCpp generatorRosCpp, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        resolveTags(taggingResolver, componentInstanceSymbol);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }

    public static boolean rosConnectionsValid(ExpandedComponentInstanceSymbol instanceSymbol) {
        AtomicBoolean result = new AtomicBoolean(true);

        instanceSymbol.getPortsList().stream()
                .filter(PortSymbol::isRosPort)
                .forEach(p ->{
                    RosConnectionSymbol rosConnectionSymbol = (RosConnectionSymbol) p.getMiddlewareSymbol().get();

                    if(!rosConnectionSymbol.getTopicName().isPresent()){
                        result.set(false);
                        Log.error("0x9d80f: Port " + p.getFullName() + " has RosConnection but no topic name!");
                    }

                    if(!rosConnectionSymbol.getTopicType().isPresent()){
                        result.set(false);
                        Log.error("0x2cc67: Port " + p.getFullName() + " has RosConnection but no topic type!");
                    }
                });

        return result.get();
    }
}
