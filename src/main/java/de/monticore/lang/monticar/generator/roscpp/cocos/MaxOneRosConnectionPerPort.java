package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class MaxOneRosConnectionPerPort implements EmbeddedMontiArcMathSymtabCoCo {
    @Override
    public void check(TaggingResolver taggingResolver, Symbol symbol) {
        if (taggingResolver == null) {
            Log.error("taggingResolver needs to be set!");
        }
        if (symbol.isKindOf(PortSymbol.KIND)) {
            check(taggingResolver, (PortSymbol) symbol);
        }

    }

    private void check(TaggingResolver taggingResolver, PortSymbol portSymbol) {
        Collection<TagSymbol> tags = taggingResolver.getTags(portSymbol, RosConnectionSymbol.KIND);
        if (tags.size() > 1) {
            Log.error("Only one RosConnection is allowed per port but " + portSymbol.getFullName() + " has " + tags.size());
        }
    }
}
