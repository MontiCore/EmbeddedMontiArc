/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolReference;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.List;
import java.util.Optional;

import static de.monticore.numberunit.Rationals.doubleToRational;

public class EmbeddedMontiArcDynamicSymbolTableHelper {


    public static void createInstance(String name, ASTSubComponent node,
                                      EMADynamicComponentSymbolReference componentTypeReference,
                                      List<ValueSymbol<TypeReference<TypeSymbol>>> configArguments,
                                      EmbeddedMontiArcDynamicSymbolTableCreator symbolTableCreator,
                                      ASTSubComponentInstance nodeInstance) {

        String dynStatic = EMADynamicPortHelper.getINSTANCE().doUnitNumberResolution(nodeInstance.getUnitNumberResolutionOpt(), symbolTableCreator);
        String dynMax = EMADynamicPortHelper.getINSTANCE().doUnitNumberResolution(nodeInstance.getDynamicNumberOfInstancesOpt(), symbolTableCreator);


        int dynStaticNumber = EMADynamicPortHelper.getINSTANCE().getNumberIfIntLiteralIsPresent(nodeInstance.getUnitNumberResolutionOpt());
        int dynMaxNumber = EMADynamicPortHelper.getINSTANCE().getNumberIfIntLiteralIsPresent(nodeInstance.getDynamicNumberOfInstancesOpt());

        EMADynamicComponentInstantiationSymbol instance = null;
        if(nodeInstance.isPresentUnitNumberResolution()){
            //ist array
            if(nodeInstance.isPresentDynamicNumberOfInstances()){
                instance = new EMADynamicComponentInstantiationSymbol(name, componentTypeReference, dynStatic, dynMax);
                instance.setNonDynamicDimension(dynStaticNumber);
                instance.setDimension(dynMaxNumber);
                instance.setDynamic(true);

                if(EMADynamicPortHelper.getINSTANCE().numberIsInfinite(nodeInstance.getDynamicNumberOfInstancesOpt())) {
                    instance.setDimensionInfinite(true);
                    instance.setDimension(dynStaticNumber);
                }

            }else{
                instance = new EMADynamicComponentInstantiationSymbol(name, componentTypeReference, dynStatic);
                instance.setDimension(dynStaticNumber);
            }
            instance.setArray(true);
        }else{
            instance = new EMADynamicComponentInstantiationSymbol(name, componentTypeReference);
        }

        Log.debug(instance.toString(), "Test");

        final EMADynamicComponentInstantiationSymbol finstance = instance;

        for (ValueSymbol<TypeReference<TypeSymbol>> valueSymbol : configArguments)
            configArguments.forEach(v -> finstance.addConfigArgument(v));

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


    public static int handleSizeResolution(ASTSubComponent node, int index, EmbeddedMontiArcDynamicSymbolTableCreator stc) {
//        int counter = 0;
        if (node.getType() instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType stype = ((ASTSimpleReferenceType) node.getType());
            if (stype.getTypeArgumentsOpt().isPresent() && stype.getTypeArguments().getTypeArgumentList().size() > index) {

                ASTTypeArgument arg = stype.getTypeArguments().getTypeArgument(index);
                if (arg instanceof ASTUnitNumberTypeArgument) {
                    Log.debug("" + ((ASTUnitNumberTypeArgument) arg).getNumberWithUnit().getNumber().get().intValue(), "New Resolution Value:");

                    return ((ASTUnitNumberTypeArgument) arg).getNumberWithUnit().getNumber().get().intValue();
                }
            }
        }
        return -1;
    }


    public static void setActualResolutionDeclaration(ASTSubComponent node, EMAComponentSymbolReference emaComponentSymbolReference, EmbeddedMontiArcDynamicSymbolTableCreator stc) {

        if((emaComponentSymbolReference.getResolutionDeclarationSymbols().size() > 0) && (node.getType() instanceof ASTSimpleReferenceType)){
            ASTSimpleReferenceType stype = ((ASTSimpleReferenceType) node.getType());

            for(int resIdx = 0; resIdx < emaComponentSymbolReference.getResolutionDeclarationSymbols().size(); ++resIdx){
                String name = emaComponentSymbolReference.getResolutionDeclarationSymbols().get(resIdx).getNameToResolve();
                int size = 0;
                if(emaComponentSymbolReference.getReferencedComponent().isPresent() && emaComponentSymbolReference.getReferencedComponent().get().getAstNode().isPresent()){
                    ASTComponent astCs = (ASTComponent) emaComponentSymbolReference.getReferencedComponent().get().getAstNode().get();

                    int index = 0;
                    for(index = 0; index < astCs.getGenericTypeParameters().getTypeVariableDeclaration2List().size(); ++index){
                        if(astCs.getGenericTypeParameters().getTypeVariableDeclaration2(index).getResolutionDeclarationOpt().isPresent() &&
                                astCs.getGenericTypeParameters().getTypeVariableDeclaration2(index).getResolutionDeclaration().getName() == name){
                            size = EmbeddedMontiArcDynamicSymbolTableHelper.handleSizeResolution(node, index, stc);
                            break;
                        }
                    }

                    if(size > 0){
                        ((ASTUnitNumberResolution) emaComponentSymbolReference.getResolutionDeclarationSymbols().get(resIdx).getASTResolution()).setNumber(Double.valueOf(size));
                    }
                }
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


    public static void fixResolutionsInSubComponents(EmbeddedMontiArcDynamicSymbolTableCreator stc, EMAComponentSymbolReference componentTypeReference){

        for(EMAComponentInstantiationSymbol instSym : componentTypeReference.getSubComponents()){

            EMAComponentSymbolReference instRef = instSym.getComponentType();
            for(int resIdx = 0; resIdx < instRef.getResolutionDeclarationSymbols().size(); ++resIdx){
                String name = instRef.getResolutionDeclarationSymbols().get(resIdx).getNameToResolve();
                ASTComponent astCs = (ASTComponent) instRef.getReferencedComponent().get().getAstNode().get();

                int index = 0;
                for(index = 0; index < astCs.getGenericTypeParameters().getTypeVariableDeclaration2List().size(); ++index) {
                    if (astCs.getGenericTypeParameters().getTypeVariableDeclaration2(index).getResolutionDeclarationOpt().isPresent() &&
                            astCs.getGenericTypeParameters().getTypeVariableDeclaration2(index).getResolutionDeclaration().getName() == name) {

                        ASTSimpleReferenceType stype = ((ASTSimpleReferenceType) ((ASTSubComponent)instSym.getAstNode().get()).getType());
                        if(stype.getTypeArgumentsOpt().isPresent()) {
                            ASTTypeArgument arg = stype.getTypeArguments().getTypeArgument(index);

                            if (arg instanceof ASTElementType) {
                                Optional<ResolutionDeclarationSymbol> rds = componentTypeReference.getResolutionDeclarationSymbol(((ASTElementType) arg).getName());
                                if (rds.isPresent() && rds.get().getASTResolution() instanceof ASTUnitNumberResolution) {
                                    int newValue = ((ASTUnitNumberResolution) rds.get().getASTResolution()).getNumberWithUnit().getNumber().get().intValue();
                                    ((ASTUnitNumberResolution) instRef.getResolutionDeclarationSymbols().get(resIdx).getASTResolution()).setNumber(Double.valueOf(newValue));
                                }
                            }
                        }
                    }
                }
            }

            instRef.fixResolutions(stc);
            fixResolutionsInSubComponents(stc, instRef);

        }


    }


}
