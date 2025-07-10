/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class TagHelper {
    private TagHelper() {
    }

    public static Map<EMAPortSymbol, RosConnectionSymbol> resolveTags(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) {
        Map<EMAPortSymbol, RosConnectionSymbol> rosConnectionSymbols = new HashMap<>();
            componentInstanceSymbol.getPortInstanceList().forEach(p -> {
                Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, RosConnectionSymbol.KIND);
                if (tmpTags.size() == 1) {
                    rosConnectionSymbols.put(p, (RosConnectionSymbol) tmpTags.iterator().next());
                }
            });
            componentInstanceSymbol.getSubComponents().forEach(sc -> rosConnectionSymbols.putAll(TagHelper.resolveTags(taggingResolver, sc)));
        return rosConnectionSymbols;
    }

    public static List<File> resolveAndGenerate(GeneratorRosCpp generatorRosCpp, TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        resolveTags(taggingResolver, componentInstanceSymbol);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }

    public static boolean rosConnectionsValid(EMAComponentInstanceSymbol instanceSymbol, boolean ros2mode) {
        AtomicBoolean result = new AtomicBoolean(true);

        instanceSymbol.getPortInstanceList().stream()
                .filter(EMAPortSymbol::isRosPort)
                .forEach(p ->{
                    RosConnectionSymbol rosConnectionSymbol = (RosConnectionSymbol) p.getMiddlewareSymbol().get();

                    if(!rosConnectionSymbol.getTopicName().isPresent()){
                        result.set(false);
                        Log.error("0x9d80f: Port " + p.getFullName() + " has RosConnection but no topic name!");
                        return;
                    }

                    if(!rosConnectionSymbol.getTopicType().isPresent()){
                        result.set(false);
                        Log.error("0x2cc67: Port " + p.getFullName() + " has RosConnection but no topic type!");
                        return;
                    }

                    String topicType = rosConnectionSymbol.getTopicType().get();
                    if (ros2mode && !topicType.contains("/msg/")) {
                        List<String> parts = new ArrayList<>(Arrays.asList(topicType.split("/")));
                        parts.add(parts.size() - 1, "msg");
                        String fixed = String.join("/", parts);
                        Log.warn("The port " + p.getFullName() + " has a Ros Topic without a msg qualifier. Was " + topicType + ", will now try with " + fixed);
                        rosConnectionSymbol.setTopicType(fixed);
                    }
                });

        return result.get();
    }
}
