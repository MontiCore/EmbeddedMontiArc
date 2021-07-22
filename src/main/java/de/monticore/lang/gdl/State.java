package de.monticore.lang.gdl;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Collectors;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualTreeBidiMap;
import org.sosy_lab.java_smt.api.BooleanFormula;
import org.sosy_lab.java_smt.api.BooleanFormulaManager;
import org.sosy_lab.java_smt.api.IntegerFormulaManager;
import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;

import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameRelation;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.se_rwth.commons.logging.Log;

public class State {
    
    private IntegerFormulaManager imgr;
    private BooleanFormulaManager bmgr;
    private BidiMap<String, Integer> constValueMap;
    private Map<String, BooleanFormula> dnfTermsByConstFunction = new TreeMap<>();
    private String identifier;

    public State(IntegerFormulaManager imgr, BooleanFormulaManager bmgr, BidiMap<String, Integer> constValueMap, String identifier) {
        this.imgr = imgr;
        this.bmgr = bmgr;
        this.constValueMap = constValueMap;
        this.identifier = identifier;
    }

    public BooleanFormula getConstraint() {
        return bmgr.and(dnfTermsByConstFunction.values());
    }

    public void addRelation(ASTGameExpression expression) {
        if (expression.getType() instanceof ASTGameFunction) {
            // expression defines new constant
            // type cast to function definition
            ASTGameFunction function = (ASTGameFunction) expression.getType();
            // type cast to values
            List<ASTGameValue> values = expression.getArgumentsList().stream()
                .map(arg -> (ASTGameValue) arg)
                .collect(Collectors.toList());
            
            // constant function argument at position n equals constant given at position n
            List<BooleanFormula> equations = new ArrayList<>(values.size());
            
            // map each value to argument position with function name
            for (int i = 0; i < values.size(); i++) {
                IntegerFormula constVariable = imgr.makeVariable(function.getFunction() + "_arg_" + this.identifier + "_" + i);
                String constValue = values.get(i).getValue();
                IntegerFormula constValueInt;

                if (constValueMap.containsKey(constValue)) {
                    constValueInt = imgr.makeNumber(constValueMap.get(constValue));
                } else {
                    constValueInt = imgr.makeNumber(constValueMap.size());
                    constValueMap.put(constValue, constValueMap.size());
                }

                equations.add(imgr.equal(constVariable, constValueInt));
            }

            BooleanFormula constSubTerm = bmgr.and(equations);
            BooleanFormula dnf;
            if (dnfTermsByConstFunction.containsKey(function.getFunction())) {
                dnf = dnfTermsByConstFunction.get(function.getFunction());
                dnfTermsByConstFunction.put(function.getFunction(), bmgr.or(dnf, constSubTerm));
            } else {
                dnf = constSubTerm;
                dnfTermsByConstFunction.put(function.getFunction(), dnf);
            }
        }
    }

}
