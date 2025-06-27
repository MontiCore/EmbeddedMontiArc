/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.*;

public class MqttTagHelper {
    private MqttTagHelper() {
    }

    public static Map<EMAPortSymbol, MqttConnectionSymbol> resolveTags(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) {
        Map<EMAPortSymbol, MqttConnectionSymbol> mqttConnectionSymbols = new HashMap<>();
            componentInstanceSymbol.getPortInstanceList().forEach(p -> {
                Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, MqttConnectionSymbol.KIND);
                if (tmpTags.size() == 1) {
                    mqttConnectionSymbols.put(p, (MqttConnectionSymbol) tmpTags.iterator().next());
                }
            });
            componentInstanceSymbol.getSubComponents().forEach(sc -> mqttConnectionSymbols.putAll(MqttTagHelper.resolveTags(taggingResolver, sc)));
        return mqttConnectionSymbols;
    }
}
