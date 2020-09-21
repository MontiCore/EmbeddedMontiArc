/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.helper;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.embeddedmontiarc.helper.EMAJavaHelper;
import de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.builder.EMADynamicEventHandlerInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTComponentEvent;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTPortResolutionDeclaration;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventSymbolTableCreator;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarSymbolFactory;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.symboltable.Scope;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;


public class ComponentEventSymbolHelper {

    private final static MontiCarSymbolFactory jSymbolFactory = new MontiCarSymbolFactory();

    //<editor-fold desc="Single Instance">

    protected static ComponentEventSymbolHelper INSTANCE = null;

    public static ComponentEventSymbolHelper getINSTANCE() {
        if (INSTANCE == null) {
            ComponentEventSymbolHelper.init();
        }
        return INSTANCE;
    }

    protected void setINSTANCE(ComponentEventSymbolHelper builder) {
        INSTANCE = builder;
    }

    public static ComponentEventSymbolHelper init() {
        ComponentEventSymbolHelper b = new ComponentEventSymbolHelper();
        b.setINSTANCE(b);
        return b;
    }

    //</editor-fold>

    public List<MCTypeSymbol> addTypeParametersToType(ComponentEventSymbol typeSymbol,
                                                      ASTTypeParameters2 astTypeParameters,
                                                      Scope currentScope) {

        for (ASTTypeVariableDeclaration2 astTypeParameter : astTypeParameters.getTypeVariableDeclaration2List()) {
//            System.out.println(astTypeParameter);
            if (astTypeParameter.getResolutionDeclarationOpt().isPresent()) {
                if(astTypeParameter.getResolutionDeclaration() instanceof ASTPortResolutionDeclaration){
                    MontiCarTypeSymbol javaTypeVariableSymbol = jSymbolFactory.createTypeVariable(astTypeParameter.getResolutionDeclaration().getName());

                    typeSymbol.addFormalTypeParameter(javaTypeVariableSymbol);
                }
                else{
                    Log.debug("0xADTYPA Case not handled", "ComponentEventSymbolHelper.addTypeParametersToType Implementation Missing");
                    Log.debug(astTypeParameter.getResolutionDeclaration().toString(), "Resolution Declaration");
                }
            }
        }
                return null;
    }

    public void setParametersOfEvent(final ComponentEventSymbol eventSymbol, ASTComponentEvent cmp, EventSymbolTableCreator symbolTableCreator) {

        for (ASTParameter astParameter : cmp.getParameterList()) {
            final String paramName = astParameter.getNameWithArray().getName();
            Log.debug(astParameter.toString(), "ASTParam");

            int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astParameter.getType());

            MCTypeReference<? extends MCTypeSymbol> paramTypeSymbol = new MontiCarTypeSymbolReference(
                    TypesPrinter.printTypeWithoutTypeArgumentsAndDimension(astParameter
                            .getType()),
                    symbolTableCreator.currentScope().get(), dimension);

            eventSymbol.addParameter(astParameter);

//            System.out.println("plaplaplapl");
        }
    }

        }
