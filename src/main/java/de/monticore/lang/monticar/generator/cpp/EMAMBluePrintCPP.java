/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class EMAMBluePrintCPP extends EMAMBluePrint {
    private List<String> additionalUserIncludeStrings = new ArrayList<>();
    private List<String> additionalStandardIncludeStrings = new ArrayList<>();
    private List<String> additionalStandardIncludeStringsWithH = new ArrayList<>();
    private List<String> additionalStandardIncludeStringsWithHPP = new ArrayList<>();
    private List<String> additionalNameSpaceStrings = new ArrayList<>();
    private List<String> additionalTypeDefStrings = new ArrayList<>();
//    public List<String> cvIncludeStrings = new ArrayList<>();
    private Method constructor;

    public EMAMBluePrintCPP(String name) {
        super(name);
    }


    public List<String> getAdditionalUserIncludeStrings() {
        return additionalUserIncludeStrings;
    }

    public void addAdditionalUserIncludeStrings(String includeString) {
        if (!hasAdditionalUserIncludeStrings(includeString))
            additionalUserIncludeStrings.add(includeString);
    }

    public boolean hasAdditionalUserIncludeStrings(String includeString) {
        return additionalUserIncludeStrings.contains(includeString);
    }

    public List<String> getAdditionalStandardIncludeStrings() {
        return additionalStandardIncludeStrings;
    }

    public void addAdditionalStandardInclude(String includeString) {
        if (!this.additionalStandardIncludeStrings.contains(includeString))
        this.additionalStandardIncludeStrings.add(includeString);
    }

    public List<String> getAdditionalStandardIncludeStringsWithH() {
        return additionalStandardIncludeStringsWithH;
    }

    public void addAdditionalStandardIncludeStringWithH(String includeString) {
        if (!this.additionalStandardIncludeStringsWithH.contains(includeString))
        this.additionalStandardIncludeStringsWithH.add(includeString);
    }

    public void addAdditionalStandardIncludeStringWithHPP(String includeString) {
        if (!this.additionalStandardIncludeStringsWithHPP.contains(includeString))
            this.additionalStandardIncludeStringsWithHPP.add(includeString);
    }

    public List<String> getAdditionalStandardIncludeStringsWithHPP() {
        return additionalStandardIncludeStringsWithHPP;
    }

    public List<String> getAdditionalNameSpaceStrings() {
        return additionalNameSpaceStrings;
    }

    public void addAdditionalNameSpaceStrings(String nameSpaceString) {
        this.additionalNameSpaceStrings.add(nameSpaceString);
    }

    public List<String> getAdditionalTypeDefStrings() {
        return additionalTypeDefStrings;
    }

    public void addAdditionalTypeDefString(String typeDefString) {
        this.additionalTypeDefStrings.add(typeDefString);
    }

    public List<String> getConsts() {
        List<String> consts = new ArrayList<>();
        for (Variable variable : genericsVariableList) {

            String defineString = "const int ";
            defineString += variable.getName();
            defineString += " = " + variable.getConstantValue() + ";\n";
            consts.add(defineString);
        }
        return consts;
    }

    public void addDefineGenerics(EMAComponentInstanceSymbol componentSymbol) {
        if (componentSymbol.getInstanceInformation().isPresent()) {
            int index = 0;
            Log.info(componentSymbol.getName(), "HasInstanceInformation:");
            for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : componentSymbol.getResolutionDeclarationSymbols()) {
                Log.info(resolutionDeclarationSymbol.getNameToResolve(), "ResDecl:");
                ASTSubComponent subComponent = componentSymbol.getInstanceInformation().get().getASTSubComponent();
                int number = InstanceInformation.getInstanceNumberFromASTSubComponent(subComponent, index);
                //if(resolutionDeclarationSymbol.getNameToResolve().equals("targetEigenvectors")){
                Log.info(subComponent.toString(), "InfoKK:");
                //}
                if (number == -1) {
                    // try with ast
                    if (resolutionDeclarationSymbol.getASTResolution() instanceof ASTUnitNumberResolution) {
                        number = ((ASTUnitNumberResolution) resolutionDeclarationSymbol.getASTResolution()).getNumber().get().intValue();
                    } else {
                        Log.info(subComponent.toString(), "No number added for" + resolutionDeclarationSymbol.getNameToResolve());
                        ++index;
                        break;
                    }
                }
                fixSubComponentInstanceNumbers(componentSymbol, resolutionDeclarationSymbol.getNameToResolve(), number, index);
                Variable constVar = new Variable();
                constVar.setName(resolutionDeclarationSymbol.getNameToResolve());
                constVar.setConstantValue("" + number);
                addGenericVariable(constVar);
                ++index;

            }
        }
    }

    public void fixSubComponentInstanceNumbers(EMAComponentInstanceSymbol componentInstanceSymbol, String nameToResolve, int numberToSet, int index) {
        for (EMAComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {

            for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : instanceSymbol.getResolutionDeclarationSymbols()) {
                Log.info(resolutionDeclarationSymbol.getNameToResolve(), "ResDeclFix:");
                ASTSubComponent subComponent = instanceSymbol.getInstanceInformation().get().getASTSubComponent();
                int number = InstanceInformation.getInstanceNumberFromASTSubComponent(subComponent, index);
                if (number == -1) {
                    //if(resolutionDeclarationSymbol.getNameToResolve().equals(nameToResolve))
                    {
                        Log.info(resolutionDeclarationSymbol.getNameToResolve()+"set number to "+numberToSet+"; oldValue: "+number, "Fixed");
                        InstanceInformation.setInstanceNumberInASTSubComponent(subComponent, nameToResolve, numberToSet);
                        break;
                    }/*else{
                        Log.info("realName:"+resolutionDeclarationSymbol.getNameToResolve() +" nameToResolve:"+nameToResolve,"Not fixing:");
                    }*/
                }
                ++index;
            }
        }
    }

    public void addInstructionToMethod(Instruction instruction, String methodName) {
        getMethod(methodName).get().addInstruction(instruction);
    }

    public void setConstructor(Method constructor) {
        this.constructor = constructor;
    }

    public Method getConstructor() {
        return constructor;
    }
}
