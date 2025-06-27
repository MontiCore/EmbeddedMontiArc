/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.EMAMBluePrint;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.SourcePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LoggingViewModel extends ViewModelBase {
    public List<Variable> variables;
    public List<EMAComponentInstanceSymbol> instanceStack;
    public EMAComponentInstanceSymbol originalSymbol;


    public List<Variable> getVariables() {
        return variables;
    }

    public List<EMAComponentInstanceSymbol> getInstanceStack() {
        return instanceStack;
    }

    public EMAComponentInstanceSymbol getOriginalSymbol() {
        return originalSymbol;
    }

    public static LoggingViewModel fromBluePrint(EMAMBluePrint bluePrint){
        LoggingViewModel res = new LoggingViewModel();
        res.originalSymbol = bluePrint.getOriginalSymbol();
        res.variables = bluePrint.getVariables().stream()
                .filter(v -> v.hasAdditionalInformation(Variable.ORIGINPORT))
                .collect(Collectors.toList());

        res.instanceStack = new ArrayList<>();
        EMAComponentInstanceSymbol curSym = bluePrint.getOriginalSymbol();
        while(curSym != null){
            res.instanceStack.add(curSym);
            curSym = curSym.getParent().orElse(null);
        }
        return res;
    }

    public int getLastLineOfInstance(){
        return originalSymbol.getComponentType()
                .getReferencedSymbol()
                .getAstNode()
                .map(ASTNode::get_SourcePositionEnd)
                .map(SourcePosition::getLine)
                .orElse(1000);
    }

    public String getTypeStringForVar(Variable var){
        String varMontiCoreType = "var";
        if(var.hasAdditionalInformation(Variable.INCOMING)){
            varMontiCoreType = "in";
        }else if(var.hasAdditionalInformation(Variable.OUTGOING)){
            varMontiCoreType = "out";
        }
        return varMontiCoreType;
    }
}
