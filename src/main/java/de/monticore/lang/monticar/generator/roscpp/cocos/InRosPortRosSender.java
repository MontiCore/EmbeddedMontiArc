package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

public class InRosPortRosSender implements EmbeddedMontiArcMathSymtabCoCo {

    @Override
    public void check(TaggingResolver taggingResolver, Symbol symbol) {
        if (symbol.isKindOf(ExpandedComponentInstanceSymbol.KIND)) {
            check(taggingResolver, (ExpandedComponentInstanceSymbol) symbol);
        }
    }

    private void check(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol symbol) {
        if (taggingResolver == null)
            throw new RuntimeException("taggingResolver must be set!");

        symbol.getConnectors().forEach(connectorSymbol -> {

            PortSymbol source = taggingResolver.<PortSymbol>resolve(symbol.getFullName() + "." + connectorSymbol.getSource(), PortSymbol.KIND).get();
            PortSymbol target = taggingResolver.<PortSymbol>resolve(symbol.getFullName() + "." + connectorSymbol.getTarget(), PortSymbol.KIND).get();

            RosConnectionSymbol sourceTag = (RosConnectionSymbol) taggingResolver.getTags(source, RosConnectionSymbol.KIND).stream()
                    .findFirst().orElse(null);
            RosConnectionSymbol targetTag = (RosConnectionSymbol) taggingResolver.getTags(target, RosConnectionSymbol.KIND).stream()
                    .findFirst().orElse(null);

            if (targetTag != null) {
                if (sourceTag != null) {
                    if (!targetTag.getTopicName().equals(sourceTag.getTopicName()))
                        Log.error("Connector between ros ports: topic name mismatch: "
                                + source.getFullName() + " has " + sourceTag.getTopicName() + " and "
                                + target.getFullName() + " has " + targetTag.getTopicName());

                    if (!targetTag.getTopicType().equals(sourceTag.getTopicType())) {
                        Log.error("Connector between ros ports: topic type mismatch: "
                                + source.getFullName() + " has " + sourceTag.getTopicType() + " and "
                                + target.getFullName() + " has " + targetTag.getTopicType());
                    }
                } else {
                    Log.error("Connector: target is ros port but source " + source.getFullName() + " is not!");
                }

            }

        });
    }
}
