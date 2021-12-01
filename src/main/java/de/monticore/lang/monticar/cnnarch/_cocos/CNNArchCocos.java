/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchNode;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.se_rwth.commons.logging.Log;

//check all cocos
public class CNNArchCocos {

    public static void checkAll(ArchitectureSymbol architecture){
        ASTCNNArchNode node = (ASTCNNArchNode) architecture.getAstNode().get();
        int findings = Log.getFindings().size();
        createASTChecker().checkAll(node);
        if (findings == Log.getFindings().size()) {
            createCNNArchPreResolveSymbolChecker().checkAll(architecture);
            if (findings == Log.getFindings().size()) {
                architecture.resolve();
                if (findings == Log.getFindings().size()) {
                    createCNNArchPostResolveSymbolChecker().checkAll(architecture);
                }
            }
        }
    }

    public static void checkAll(CNNArchCompilationUnitSymbol compilationUnit){
        ASTCNNArchNode node = (ASTCNNArchNode) compilationUnit.getAstNode().get();
        int findings = Log.getFindings().size();
        createASTChecker().checkAll(node);
        if (findings == Log.getFindings().size()) {
            createCNNArchPreResolveSymbolChecker().checkAll(compilationUnit);
            if (findings == Log.getFindings().size()) {
                compilationUnit.getArchitecture().resolve();
                if (findings == Log.getFindings().size()) {
                    createCNNArchPostResolveSymbolChecker().checkAll(compilationUnit);
                }
            }
        }
    }

    //checks cocos based on symbols after the resolve method of the ArchitectureSymbol is called
    public static CNNArchSymbolCoCoChecker createCNNArchPostResolveSymbolChecker() {
        return new CNNArchSymbolCoCoChecker()
                .addCoCo(new CheckIOType())
                .addCoCo(new CheckIOArrayLength())
                .addCoCo(new CheckElementInputs())
                .addCoCo(new CheckIOAccessAndIOMissing())
                .addCoCo(new CheckArchitectureFinished())
                .addCoCo(new CheckVariableMember())
                .addCoCo(new CheckLayerVariableDeclarationLayerType())
                .addCoCo(new CheckLayerVariableDeclarationIsUsed())
                .addCoCo(new CheckConstants())
                .addCoCo(new CheckLargeMemoryLayer())
                .addCoCo(new CheckEpisodicMemoryLayer())
                .addCoCo(new CheckUnrollInputsOutputsTooMany())
                .addCoCo(new CheckAdaNetMalFormedArchitecture());
    }

    //checks cocos based on symbols before the resolve method of the ArchitectureSymbol is called
    public static CNNArchSymbolCoCoChecker createCNNArchPreResolveSymbolChecker() {
        return new CNNArchSymbolCoCoChecker()
                .addCoCo(new CheckVariableDeclarationName())
                .addCoCo(new CheckVariableName())
                .addCoCo(new CheckArgmaxLayer())
                .addCoCo(new CheckExpressions())
                .addCoCo(new CheckAdaNetTooManyLayers());
                //.addCoCo(new CheckAdaNetPathToFilesExists())
    }

    //checks all normal cocos
    public static CNNArchCoCoChecker createASTChecker() {
        return new CNNArchCoCoChecker()
                .addCoCo(new CheckLayer())
                .addCoCo(new CheckRangeOperators())
                .addCoCo(new CheckParameterName())
                .addCoCo(new CheckLayerName())
                .addCoCo(new CheckArgument())
                .addCoCo(new CheckLayerRecursion());
    }
}