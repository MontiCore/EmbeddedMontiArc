/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ActualTypeArgumentNode;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventReferenceExpressionSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.types.TypesHelper;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.monticore.types.types._ast.ASTWildcardType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class EventTypeHelper {



    public static void addTypeArgumentsToEventReferenceExpression(EventReferenceExpressionSymbol eres, ASTType astType, final Scope currentScope){
        if(astType == null){
            return;
        }

        if(astType instanceof ASTSimpleReferenceType){
            ASTSimpleReferenceType astSimpleReferenceType = (ASTSimpleReferenceType) astType;
            if (!astSimpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                return ;
            }
            
            for (ASTTypeArgument astTypeArgument : astSimpleReferenceType.getTypeArgumentsOpt().get()
                    .getTypeArgumentList()) {

                if (astTypeArgument instanceof ASTType) {

//                    System.out.println(astTypeArgument.getClass().toString()+" is ASTType");
                    ASTType astTypeNoBound = (ASTType) astTypeArgument;
                    int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astTypeNoBound);
                    MCTypeReference<? extends MCTypeSymbol> typeArgumentSymbolReference = new MontiCarTypeSymbolReference(
                            ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astTypeNoBound),
                            currentScope, dimension);

                    eres.addActualTypeArgument(new ActualTypeArgumentNode(typeArgumentSymbolReference,astTypeNoBound));

                } else {
                    Log.error("0xU0401 Unknown type arguments " + astTypeArgument + " of type ");
                }

            }
        }else{
            Log.error("Unsupported ast type! " + astType.getClass().toString());
        }
    }

}
