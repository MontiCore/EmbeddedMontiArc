/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTAdaptableKeyword;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.resolution._ast.ASTResolutionDeclaration;
import de.monticore.lang.monticar.resolution._ast.ASTTypeNameResolutionDeclaration;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbolReference;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.List;

import static de.monticore.numberunit.Rationals.doubleToRational;


/**
 * @author Sascha Schneiders
 */
public class EmbeddedMontiArcSymbolTableHelper {


    public static void doSubComponentInstanceResolution(ASTSubComponentInstance node,
                                                        ComponentSymbolReference componentSymbolReference,
                                                        EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {

        if (node.getUnitNumberResolutionOpt().isPresent()) {
            ASTUnitNumberResolution unitNumberResolution = node.getUnitNumberResolution();
            ASTNumberWithUnit toSet = null;
            if (unitNumberResolution.getNumberWithUnitOpt().isPresent()) {
                toSet = unitNumberResolution.getNumberWithUnit();

            } else if (unitNumberResolution.getNameOpt().isPresent()) {

                ResolutionDeclarationSymbol resDeclSym = symbolTableCreator.
                        componentStack.peek()
                        .getResolutionDeclarationSymbol(unitNumberResolution.getNameOpt().get()).get();
                Log.debug(resDeclSym.getASTResolution().toString(), "Found ResolutionDeclarationSymbol:");
                toSet = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getNumberWithUnit();

                Log.debug("" + toSet.getNumber().get().intValue(), "ToSet Number:");
            }
            node.getUnitNumberResolution().setUnit(toSet.getUnit());
            node.getUnitNumberResolution().setNumber(toSet.getNumber().get());

            Log.debug("" + node.getUnitNumberResolution().getNumber().get().intValue(),
                    "SubComponentResolution Number:");
        }
    }

    public static void setActualResolutionDeclaration(ASTSubComponent node,
                                                      ComponentSymbolReference componentSymbolReference) {
        int index = 0;
        int size = EMAPortHelper.handleSizeResolution(node, index);
        if (size > 0 && componentSymbolReference.getResolutionDeclarationSymbols().size() > 0) {
            if (componentSymbolReference.getResolutionDeclarationSymbols().get(index)
                    .getASTResolution() instanceof ASTUnitNumberResolution) {
                Log.debug(size + "", "Set new Resolution");
                ((ASTUnitNumberResolution) componentSymbolReference.getResolutionDeclarationSymbols()
                        .get(index).getASTResolution()).setNumber(Double.valueOf(size));
            }
        } else {
            for (int i = 0; i < componentSymbolReference.getResolutionDeclarationSymbols().size(); ++i) {
                Rational numberToSetTo = doubleToRational(((ASTUnitNumberResolution) componentSymbolReference
                        .getReferencedSymbol().getResolutionDeclarationSymbols().get(i).getASTResolution())
                        .getNumber().get());
                ((ASTUnitNumberResolution) componentSymbolReference.getResolutionDeclarationSymbols().get(i)
                        .getASTResolution()).setNumber(numberToSetTo.doubleValue());
            }
        }
    }

    /**
     * Creates the instance and adds it to the symTab.
     */

    public static void createInstance(String name, ASTSubComponent node,
                                      ComponentSymbolReference componentTypeReference,
                                      List<ValueSymbol<TypeReference<TypeSymbol>>> configArguments,
                                      EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        ComponentInstanceSymbol instance = new ComponentInstanceSymbol(name,
                componentTypeReference);
        for (ValueSymbol<TypeReference<TypeSymbol>> valueSymbol : configArguments)
            configArguments.forEach(v -> instance.addConfigArgument(v));
        // create a subscope for the instance
        symbolTableCreator.addToScopeAndLinkWithNode(instance, node);
        Log.debug(symbolTableCreator.currentScope().get().toString(),
                "SubComponentInstance Scope");
        // remove the created instance's scope
        symbolTableCreator.removeCurrentScope();
        InstanceInformation instanceInformation = new InstanceInformation();
        instanceInformation.setCompName(name);
        instanceInformation.setASTSubComponent(node);
        String reslString = "";
        for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : componentTypeReference
                .getResolutionDeclarationSymbols()) {
            reslString += "Name:" + resolutionDeclarationSymbol.getNameToResolve() + "value: "
                    + ((ASTUnitNumberResolution) resolutionDeclarationSymbol.getASTResolution()).getNumber()
                    .get().intValue();
        }
        Log.info(reslString, "CompInst");
        InstancingRegister.addInstanceInformation(instanceInformation);
        Log.debug(name, "created SubComponentInstance:");
    }


    public static void handleResolutionDeclaration(ComponentSymbol typeSymbol,
                                                   ASTTypeParameters2 astTypeParameters, Scope currentScope,
                                                   ASTComponent node,
                                                   EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        for (ASTTypeVariableDeclaration2 astTypeParameter : astTypeParameters.getTypeVariableDeclaration2List()) {
            if (astTypeParameter.getResolutionDeclarationOpt().isPresent() && astTypeParameter
                    .getResolutionDeclaration() instanceof ASTTypeNameResolutionDeclaration) {
                Log.debug(astTypeParameter.toString(), "Resolution Declaration:");
                ASTResolutionDeclaration astResDecl = astTypeParameter.getResolutionDeclaration();

                ResolutionDeclarationSymbolReference resDeclSymRef;
                resDeclSymRef = ResolutionDeclarationSymbolReference.constructResolutionDeclSymbolRef(
                        astResDecl.getName(),
                        ((ASTTypeNameResolutionDeclaration) astResDecl).getResolution());

                Log.debug(resDeclSymRef.getNameToResolve(),
                        "Added ResolutionDeclarationSymbol with name: ");
                typeSymbol.addResolutionDeclarationSymbol(resDeclSymRef);
                // TODO Resolution maybe link with node
                symbolTableCreator.addToScopeAndLinkWithNode(resDeclSymRef,
                        astTypeParameter);
            }
        }
    }


    public static void setParametersOfComponent(final ComponentSymbol componentSymbol, ASTComponent cmp
            , EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        Log.debug(componentSymbol.toString(), "ComponentPreParam");
        for (ASTParameter astParameter : cmp.getParameterList()) {
            final String paramName = astParameter.getNameWithArray().getName();
            Log.debug(astParameter.toString(), "ASTParam");
            int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astParameter.getType());

            // TODO enable if needed and remove line below
            MCTypeReference<? extends MCTypeSymbol> paramTypeSymbol = new MontiCarTypeSymbolReference(
                    TypesPrinter.printTypeWithoutTypeArgumentsAndDimension(astParameter
                            .getType()),
                    symbolTableCreator.currentScope().get(), dimension);

            EMATypeHelper.addTypeArgumentsToTypeSymbol(paramTypeSymbol, astParameter.getType(), symbolTableCreator);

            final MCFieldSymbol parameterSymbol = symbolTableCreator.
                    jSymbolFactory.createFormalParameterSymbol(paramName,
                    (MontiCarTypeSymbolReference) paramTypeSymbol);
            componentSymbol.addConfigParameter(parameterSymbol);
            componentSymbol.addParameter(astParameter);

            if (astParameter.adaptableKeywordIsPresent())
                addConfigPort(cmp, parameterSymbol, astParameter);
        }
        Log.debug(componentSymbol.toString(), "ComponentPostParam");
    }

    public static void addConfigPort(ASTComponent astComponent, MCFieldSymbol parameterSymbol, ASTParameter astParameter) {
        ASTPort tmpASTPort = ASTPort.getBuilder()
                .name(parameterSymbol.getName())
                .type(astParameter.getType())
                .incoming(true)
                .outgoing(false)
                .adaptableKeyword(ASTAdaptableKeyword.getBuilder().build())
                .build();

        ASTInterface tmpInterface = EmbeddedMontiArcNodeFactory.createASTInterface();
        tmpInterface.setPorts(Lists.newArrayList(tmpASTPort));

        astComponent.getBody().getElements().add(tmpInterface);
    }

    public static boolean needsInstanceCreation(ASTComponent node, ComponentSymbol symbol,
                                                EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        boolean instanceNameGiven = false;// node.getInstanceName().isPresent();
        boolean autoCreationPossible = symbol.getFormalTypeParameters().size() == 0;

        return symbolTableCreator.autoInstantiate && (instanceNameGiven ||
                autoCreationPossible);
    }
}
