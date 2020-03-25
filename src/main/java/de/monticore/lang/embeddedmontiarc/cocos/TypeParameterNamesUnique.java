/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class TypeParameterNamesUnique implements EmbeddedMontiArcASTComponentCoCo {

    /**
     * @see de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo#check(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent)
     */
    @Override
    public void check(ASTComponent node) {
        if (node.getGenericTypeParametersOpt().isPresent()) {
            ASTTypeParameters2 typeParameters = node.getGenericTypeParametersOpt().get();
            List<String> typeParameterNames = new ArrayList<>();
            for (ASTTypeVariableDeclaration2 typeParameter : typeParameters.getTypeVariableDeclaration2List()) {

                if (typeParameter.getNamingResolutionOpt().isPresent() && typeParameterNames.contains(typeParameter.getNamingResolution().getName())) {
                    Log.error(String.format(
                            "0x35F1A The formal type parameter name \"%s\" is not unique",
                            typeParameter.getNamingResolution().getName()), typeParameter.get_SourcePositionStart());
                } else {
                    //typeParameterNames.add(typeParameter.getNamingResolution().get().getName());
                }
            }
        }
    }

}
