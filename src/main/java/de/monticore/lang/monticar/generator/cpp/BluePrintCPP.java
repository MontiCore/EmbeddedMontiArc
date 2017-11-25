package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.InstanceInformation;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class BluePrintCPP extends BluePrint {
    public List<String> additionalIncludeStrings = new ArrayList<>();

    public BluePrintCPP(String name) {
        super(name);
    }


    public List<String> getAdditionalIncludeStrings() {
        return additionalIncludeStrings;
    }

    public void addAdditionalIncludeString(String includeString) {
        if (!hasAdditionalIncludeString(includeString))
            additionalIncludeStrings.add(includeString);
    }

    public boolean hasAdditionalIncludeString(String includeString) {
        return additionalIncludeStrings.contains(includeString);
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

    public void addDefineGenerics(ExpandedComponentInstanceSymbol componentSymbol) {
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
                    Log.info(subComponent.toString(), "No number added for" + resolutionDeclarationSymbol.getNameToResolve());
                    ++index;
                    break;
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

    public void fixSubComponentInstanceNumbers(ExpandedComponentInstanceSymbol componentInstanceSymbol, String nameToResolve, int numberToSet, int index) {
        for (ExpandedComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {

            for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : instanceSymbol.getResolutionDeclarationSymbols()) {
                Log.info(resolutionDeclarationSymbol.getNameToResolve(), "ResDeclFix:");
                ASTSubComponent subComponent = instanceSymbol.getInstanceInformation().get().getASTSubComponent();
                int number = InstanceInformation.getInstanceNumberFromASTSubComponent(subComponent, index);
                if (number == -1) {
                    //if(resolutionDeclarationSymbol.getNameToResolve().equals(nameToResolve))
                    {
                        Log.info(resolutionDeclarationSymbol.getNameToResolve(), "Fixed");
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
}
