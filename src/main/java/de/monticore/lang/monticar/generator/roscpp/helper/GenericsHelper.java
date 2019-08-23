/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class GenericsHelper {
    private GenericsHelper(){

    }

    public static List<String> getGenericsDefinition(EMAComponentInstanceSymbol componentSymbol) {
        List<String> res = new ArrayList<>();
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
                res.add("const int " + resolutionDeclarationSymbol.getNameToResolve() + " = " + number + ";");
                ++index;

            }
            return res.stream().sorted().collect(Collectors.toList());
        }
        return new ArrayList<>();
    }

    public static void fixSubComponentInstanceNumbers(EMAComponentInstanceSymbol componentInstanceSymbol, String nameToResolve, int numberToSet, int index) {
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

}
