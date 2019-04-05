package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class BluePrintCPP {
    public List<String> additionalIncludeStrings = new ArrayList<>();
    protected List<Variable> variables = new ArrayList<>();
    protected List<Method> methods = new ArrayList<>();
    protected List<Variable> genericsVariableList = new ArrayList<>();
    protected String name;
    protected String packageName;
    protected EMAComponentInstanceSymbol originalSymbol;

    public BluePrintCPP(String name) {
        this.name = name;
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

    public void setVariables(List<Variable> variables) {
        this.variables = variables;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setPackageName(String name) {
        this.packageName = name;
    }

    public String getPackageName() {
        return packageName;
    }

    public List<Variable> getVariables() {
        return variables;
    }

    public int howManyVariables() {
        return variables.size();
    }

    public Optional<Variable> getVariable(String variableName) {
        for (Variable v : variables)
            if (v.getName().equals(variableName))
                return Optional.of(v);
        return Optional.empty();
    }

    public void addVariable(Variable v) {
        variables.add(v);
    }

    public void removeVariable(Variable v) {
        variables.remove(v);
    }

    public void addMethod(Method method) {
        methods.add(method);
    }

    public List<Method> getMethods() {
        return methods;
    }

    public void addGenericVariable(Variable genericVariable) {
        genericsVariableList.add(genericVariable);
    }

    public List<Variable> getGenericsVariableList() {
        return genericsVariableList;
    }

    public EMAComponentInstanceSymbol getOriginalSymbol() {
        return originalSymbol;
    }

    public void setOriginalSymbol(EMAComponentInstanceSymbol originalSymbol) {
        this.originalSymbol = originalSymbol;
    }

    public Optional<Method> getMethod(String name) {
        for (Method method : getMethods()) {
            if (method.getName().equals(name))
                return Optional.of(method);
        }
        return Optional.empty();
    }
}
