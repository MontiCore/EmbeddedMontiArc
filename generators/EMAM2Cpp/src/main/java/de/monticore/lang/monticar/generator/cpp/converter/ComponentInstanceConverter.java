/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;
import de.monticore.lang.monticar.generator.cpp.loopSolver.EquationSystemComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.loopSolver.RHSComponentInstanceSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.ImportStatement;
import de.se_rwth.commons.logging.Log;

/**
 *
 */
public class ComponentInstanceConverter {

    public static Variable convertComponentInstanceSymbolToVariable(EMAComponentInstanceSymbol instanceSymbol, EMAComponentInstanceSymbol componentSymbol) {
        //TODO implement templating

        Log.info(componentSymbol.toString(), "Parent:");
        Log.info(instanceSymbol.toString(), "Instance:");
        for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : componentSymbol.getResolutionDeclarationsSubComponent(instanceSymbol.getName()))
            Log.info(resolutionDeclarationSymbol.getNameToResolve() + ": " + resolutionDeclarationSymbol.getASTResolution(), "ResolutionDeclarations");


        Variable variable = new Variable();
        if (componentSymbol instanceof EquationSystemComponentInstanceSymbol) {
            variable.setName(GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(instanceSymbol.getFullName()));
            variable.setVariableType(((EquationSystemComponentInstanceSymbol) componentSymbol).getTypeOfSubComponent(instanceSymbol));
        } else if (componentSymbol instanceof RHSComponentInstanceSymbol) {
            variable.setName(GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(instanceSymbol.getFullName()));
            variable.setVariableType(CPPEquationSystemHelper.getTypeOfSubComponent(instanceSymbol));
        }else {
            variable.setName(instanceSymbol.getName());
            variable.setVariableType(TypeConverter.getVariableTypeForMontiCarInstance(instanceSymbol));
        }
        //String res = getTargetCodeImportString(instanceSymbol, componentSymbol);
        //if (res != null)
        //   variable.setTargetCodeImport(res);

        if (instanceSymbol instanceof EMADynamicComponentInstanceSymbol) {
            variable.setDynamic(
                    ((EMADynamicComponentInstanceSymbol) instanceSymbol).isDynamicInstance()
            );
        }

        variable.addAdditionalInformation(Variable.COMPONENT);
        return variable;
    }


}
