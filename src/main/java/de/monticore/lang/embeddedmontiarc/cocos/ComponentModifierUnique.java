/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponentModifier;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

/**
 * ensures that component modifiers such as virtual or nondirectfeedthrough are unique
 */
public class ComponentModifierUnique implements EmbeddedMontiArcASTComponentCoCo {

    @Override
    public void check(ASTComponent node) {
        Set<Class> modifiers = new HashSet<>();
        for (ASTComponentModifier astComponentModifier : node.getComponentModifierList()) {
            if (modifiers.contains(astComponentModifier.getClass()))
                Log.error("0xAC010 Component Modifiers must be unique");
            modifiers.add(astComponentModifier.getClass());
        }
    }
}
