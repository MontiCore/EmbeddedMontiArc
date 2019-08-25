/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTPortCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

public class OnlyIncomingPortIsConfig implements EmbeddedMontiArcASTPortCoCo {

    @Override
    public void check(ASTPort node) {
        Symbol symbol = node.getSymbol().orElse(null);
        if(symbol == null) return;

        if(symbol.isKindOf(EMAPortSymbol.KIND)){
            check((EMAPortSymbol)symbol);
        }
    }

    private void check(EMAPortSymbol symbol) {
        if(symbol.isConfig() && symbol.isOutgoing())
            Log.error("0x7FF02 Config ports can only be incoming!");
    }
}
