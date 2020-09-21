package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.ConsistencyChecker;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import jline.internal.Log;

public class ComponentConsistent implements EmbeddedMontiArcDynamicASTComponentCoCo {

    @Override
    public void check(ASTComponent node) {
        if (node.getSymbolOpt().isPresent() && node.getSymbolOpt().get() instanceof EMADynamicComponentSymbol) {
            new ConsistencyChecker().Check(((EMADynamicComponentSymbol) node.getSymbolOpt().get()));
        } else {
            Log.error("component symbol not available");
        }
    }
}





