/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper;
import de.monticore.lang.monticar.ValueSymbol;
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
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.List;

import static de.monticore.numberunit.Rationals.doubleToRational;


/**
 */
public class EmbeddedMontiArcSymbolTableHelper {


    public static void doSubComponentInstanceResolution(ASTSubComponentInstance node,
                                                        EMAComponentSymbolReference emaComponentSymbolReference,
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
                                                      EMAComponentSymbolReference emaComponentSymbolReference) {
        int index = 0;
        int size = EMAPortHelper.handleSizeResolution(node, index);
        if (size > 0 && emaComponentSymbolReference.getResolutionDeclarationSymbols().size() > 0) {
            if (emaComponentSymbolReference.getResolutionDeclarationSymbols().get(index)
                    .getASTResolution() instanceof ASTUnitNumberResolution) {
                Log.debug(size + "", "Set new Resolution");
                ((ASTUnitNumberResolution) emaComponentSymbolReference.getResolutionDeclarationSymbols()
                        .get(index).getASTResolution()).setNumber(Double.valueOf(size));
            }
        } else {
            for (int i = 0; i < emaComponentSymbolReference.getResolutionDeclarationSymbols().size(); ++i) {
                Rational numberToSetTo = doubleToRational(((ASTUnitNumberResolution) emaComponentSymbolReference
                        .getReferencedSymbol().getResolutionDeclarationSymbols().get(i).getASTResolution())
                        .getNumber().get());
                ((ASTUnitNumberResolution) emaComponentSymbolReference.getResolutionDeclarationSymbols().get(i)
                        .getASTResolution()).setNumber(numberToSetTo.doubleValue());
            }
        }
    }

    /**
     * Creates the instance and adds it to the symTab.
     */

    public static void createInstance(String name, ASTSubComponent node,
                                      EMAComponentSymbolReference componentTypeReference,
                                      List<ValueSymbol<TypeReference<TypeSymbol>>> configArguments,
                                      EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        EMAComponentInstantiationSymbol instance = new EMAComponentInstantiationSymbol(name,
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


    public static void handleResolutionDeclaration(EMAComponentSymbol typeSymbol,
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


    public static void setParametersOfComponent(final EMAComponentSymbol emaComponentSymbol, ASTComponent cmp
            , EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        Log.debug(emaComponentSymbol.toString(), "ComponentPreParam");
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
            emaComponentSymbol.addConfigParameter(parameterSymbol);
            emaComponentSymbol.addParameter(astParameter);

            if (astParameter.getAdaptableKeywordOpt().isPresent())
                addConfigPort(cmp, parameterSymbol, astParameter);
        }
        Log.debug(emaComponentSymbol.toString(), "ComponentPostParam");
    }

    public static void addConfigPort(ASTComponent astComponent, MCFieldSymbol parameterSymbol, ASTParameter astParameter) {
        ASTPort tmpASTPort = EmbeddedMontiArcMill.portBuilder()
                .setName(parameterSymbol.getName())
                .setType(astParameter.getType())
                .setIncoming(true)
                .setOutgoing(false)
                .setAdaptableKeyword(EmbeddedMontiArcMill.adaptableKeywordBuilder().build())
                .build();

        ASTInterface tmpInterface = EmbeddedMontiArcNodeFactory.createASTInterface();
        tmpInterface.setPortsList(Lists.newArrayList(tmpASTPort));

        astComponent.getBody().getElementList().add(tmpInterface);
    }

    public static boolean needsInstanceCreation(ASTComponent node, EMAComponentSymbol symbol,
                                                EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        boolean instanceNameGiven = false;// node.getInstanceName().isPresent();
        boolean autoCreationPossible = symbol.getFormalTypeParameters().size() == 0;

        return symbolTableCreator.autoInstantiate && (instanceNameGiven ||
                autoCreationPossible);
    }
}
