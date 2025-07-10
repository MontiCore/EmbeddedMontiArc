/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTSubComponentCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class ReferencedSubComponentExistsEMAM implements EmbeddedMontiArcASTSubComponentCoCo {
    public ReferencedSubComponentExistsEMAM() {
    }

    public void check(ASTSubComponent node) {
        String referenceName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getType());
        Scope scope = node.getEnclosingScope();
        Optional<EMAComponentSymbol> componentSymbol = scope.resolve(referenceName, EMAComponentSymbol.KIND);
        if (!componentSymbol.isPresent()) {
            Log.error(String.format("0x069B7 Type \"%s\" could not be resolved", referenceName), node.get_SourcePositionStart());
        }

    }
}
