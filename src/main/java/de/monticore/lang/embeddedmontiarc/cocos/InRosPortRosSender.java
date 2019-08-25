/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;
import de.se_rwth.commons.logging.Log;

public class InRosPortRosSender implements EmbeddedMontiArcASTComponentCoCo {

    @Override
    public void check(ASTComponent node) {
        Symbol symbol = node.getSymbol().orElse(null);
        if(symbol.isKindOf(EMAComponentSymbol.KIND)){
            check((EMAComponentSymbol) symbol);
        }

    }

    private void check(EMAComponentSymbol symbol) {
        symbol.getConnectors().forEach(connector -> {

            EMAPortSymbol source = null;
            EMAPortSymbol target = null;
            try {
                source = connector.getSourcePort();
                target = connector.getTargetPort();
            } catch (ResolvedSeveralEntriesException ignored) {
                //needed so that other invalid coco tests dont fail(e.g. UniquePortsTest)
                Log.warn(ignored.getMessage());
            }

            if(source == null || target == null){
                Log.warn("Could not resolve target or source!");
                return;
            }

            RosConnectionSymbol sourceTag = (RosConnectionSymbol) source.getMiddlewareSymbol()
                    .filter(mws -> mws.isKindOf(RosConnectionSymbol.KIND)).orElse(null);

            RosConnectionSymbol targetTag = (RosConnectionSymbol) target.getMiddlewareSymbol()
                    .filter(mws -> mws.isKindOf(RosConnectionSymbol.KIND)).orElse(null);

            if (targetTag != null) {
                if (sourceTag != null) {
                    if (!targetTag.getTopicName().equals(sourceTag.getTopicName())) {
                        Log.error("0x23a0d Topic name mismatch: " + source.getFullName() + " and " + target.getFullName());
                    }

                    if (!targetTag.getTopicType().equals(sourceTag.getTopicType())) {
                        Log.error("0x31f6e Topic type mismatch: "+source.getFullName()+" and " +target.getFullName());
                    }
                } else {
                    Log.error("0x3830a Connector: target is ros port but source " + source.getFullName() + " is not!");
                }

            }
        });
    }
}
