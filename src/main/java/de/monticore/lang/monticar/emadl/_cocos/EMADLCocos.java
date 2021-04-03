/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.AtomicComponentCoCo;
import de.monticore.lang.math._cocos.MatrixAssignmentDeclarationCheck;
import de.monticore.lang.mathopt._cocos.MathOptCocos;
import de.monticore.lang.mathopt._cocos.OptimizationConditionCheck;
import de.monticore.lang.mathopt._cocos.OptimizationStatementCheck;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;

import java.util.Optional;

//check all cocos
public class EMADLCocos {

    public static void checkAll(EMAComponentInstanceSymbol instance){
        Optional<ArchitectureSymbol> architecture = instance.getSpannedScope().
                resolve("", ArchitectureSymbol.KIND);

        architecture.ifPresent(CNNArchCocos::checkAll);
    }

    private static EMADLCoCoChecker createASTChecker() {
        CheckBehaviorName behaviorCoco = new CheckBehaviorName();
        return new EMADLCoCoChecker()
                //EMA cocos
                .addCoCo(new UniquePorts())
                .addCoCo(new SubComponentsConnected())
                .addCoCo(new PackageLowerCase())
                .addCoCo(new ComponentCapitalized())
                .addCoCo(new DefaultParametersHaveCorrectOrder())
                .addCoCo(new ComponentWithTypeParametersHasInstance())
                .addCoCo(new TypeParameterNamesUnique())
                .addCoCo(new ParameterNamesUnique())
                .addCoCo(new TopLevelComponentHasNoInstanceName())
                .addCoCo(new ConnectorEndPointCorrectlyQualified())
                .addCoCo(new InPortUniqueSender())
                .addCoCo(new ReferencedSubComponentExists())
                .addCoCo(new PortTypeOnlyBooleanOrSIUnit())
                //EMADL cococs
                .addCoCo((EMADLASTBehaviorEmbeddingCoCo) behaviorCoco)
                .addCoCo((EMADLASTBehaviorNameCoCo) behaviorCoco)
                .addCoCo(new AtomicComponentCoCo())
                //Math cocos
                .addCoCo(new MatrixAssignmentDeclarationCheck())
                //.addCoCo(new MatrixAssignmentCheck())
                //MathOpt Cocos
                .addCoCo(new OptimizationStatementCheck())
                .addCoCo(new OptimizationConditionCheck());
    }
}
