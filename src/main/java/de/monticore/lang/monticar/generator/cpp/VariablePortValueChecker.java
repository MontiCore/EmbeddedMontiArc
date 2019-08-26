/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class VariablePortValueChecker extends Variable {

    protected static int PVCCOUNTER =0;

    protected int idPVC = 0;

    protected List<VariablePortValueCheckerTestSymbol> testsInitList = new ArrayList<>();

    public VariablePortValueChecker(String name){
        super();
        this.setName(name);
        this.idPVC = (PVCCOUNTER++);

        this.setPublic(false);
    }

    public int getIdPVC() {
        return idPVC;
    }

    @Override
    public String getNameTargetLanguageFormat() {
        String s =  "__pvc_"+this.idPVC+"_"+this.getName();
        return s.replaceAll("\\[", "_").replaceAll("\\]", "_");
    }

    public String getTypeTargetLanguageFormat(){
        return "PortValueCheck<"+this.getVariableType().getTypeNameTargetLanguage()+","+this.testsInitList.size()+">";
    }

    public void addInitInstructionsToMethod(Method init){
        TargetCodeInstruction ti = new TargetCodeInstruction(getNameTargetLanguageFormat()+".setPortReference(&"+GeneralHelperMethods.getTargetLanguageVariableInstanceName(this.getName())+");\n" );
        init.addInstruction(ti);

        int id = 0;
        for (VariablePortValueCheckerTestSymbol test : this.testsInitList){
            String s = getNameTargetLanguageFormat()+".";
            if(test.testType == VariablePortValueCheckerTestSymbolType.EqualsTest) {
                s += "set_Test_Equals";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.RangeTest){
                s += "set_Test_Range";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.GreaterTest){
                s += "set_Test_Greater";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.GreaterEqualsTest){
                s += "set_Test_GreaterEquals";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.LowerTest){
                s += "set_Test_Lower";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.LowerEqualsTest) {
                s += "set_Test_LowerEquals";
            }else if(test.testType == VariablePortValueCheckerTestSymbolType.NotEquals){
                s += "set_Test_NotEquals";
            }else{
                continue;
            }
            s += "(" + id + ", " + test.value + ");\n";

            init.addInstruction(new TargetCodeInstruction(s));

            ++id;
        }
    }

    public TargetCodeInstruction nextValueInstruction(){
        return new TargetCodeInstruction(this.getNameTargetLanguageFormat()+".next();\n");
    }

    public void addTestSymbol(VariablePortValueCheckerTestSymbol symbol){
        testsInitList.add(symbol);
    }

    public void addTestSymbol_Equals(String value){
        addTestSymbol(
                new VariablePortValueCheckerTestSymbol(value, VariablePortValueCheckerTestSymbolType.EqualsTest)
        );
    }

    public void addTestSymbol_Range(String lower, String upper){
        addTestSymbol(
                new VariablePortValueCheckerTestSymbol(lower+", "+upper, VariablePortValueCheckerTestSymbolType.RangeTest)
        );
    }

    public void addTestSymbol_Compare(String operator, String value){

        VariablePortValueCheckerTestSymbolType t = VariablePortValueCheckerTestSymbolType.EqualsTest;
        if(operator.equals(">")){
            t = VariablePortValueCheckerTestSymbolType.GreaterTest;
        }else if(operator.equals(">=")){
            t = VariablePortValueCheckerTestSymbolType.GreaterEqualsTest;
        }else if(operator.equals("<")){
            t = VariablePortValueCheckerTestSymbolType.LowerTest;
        }else if(operator.equals("<=")){
            t = VariablePortValueCheckerTestSymbolType.LowerEqualsTest;
        }else if(operator.equals("!=")){
            t = VariablePortValueCheckerTestSymbolType.NotEquals;
        }

        addTestSymbol(
                new VariablePortValueCheckerTestSymbol(value, t)
        );

    }

    //<editor-fold desc="VariablePortValueCheckerTestSymbol">

    public enum VariablePortValueCheckerTestSymbolType{
        EqualsTest,
        RangeTest,
        GreaterTest,
        GreaterEqualsTest,
        LowerTest,
        LowerEqualsTest,
        NotEquals
    }

    public class VariablePortValueCheckerTestSymbol{
        public String value;
        public VariablePortValueCheckerTestSymbolType testType;

        public VariablePortValueCheckerTestSymbol(String value, VariablePortValueCheckerTestSymbolType testType){
            this.value = value;
            this.testType = testType;
        }
    }

    //</editor-fold>
}
