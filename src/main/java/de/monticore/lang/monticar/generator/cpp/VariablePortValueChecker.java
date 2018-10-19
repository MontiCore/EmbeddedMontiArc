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
        return "__pvc_"+this.idPVC+"_"+GeneralHelperMethods.getTargetLanguageVariableInstanceName(this.getName());
        //return name.replaceAll("\\[", "_").replaceAll("\\]", "_");
    }

    public String getTypeTargetLanguageFormat(){
        return "PortValueCheck<"+this.getVariableType().getTypeNameTargetLanguage()+","+this.testsInitList.size()+">";
    }

    public void addInitInstructionsToMethod(Method init){
        TargetCodeInstruction ti = new TargetCodeInstruction(getNameTargetLanguageFormat()+".setPortReference(&"+this.getName()+");\n" );
        init.addInstruction(ti);

        int id = 0;
        for (VariablePortValueCheckerTestSymbol test : this.testsInitList){
            String s = getNameTargetLanguageFormat()+".";
            if(test.testType == VariablePortValueCheckerTestSymbolType.EqualsTest){
                s += "setEqualsTest("+id+", "+test.value+");\n";
            }else{
                continue;
            }

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

    //<editor-fold desc="VariablePortValueCheckerTestSymbol">

    public enum VariablePortValueCheckerTestSymbolType{
        EqualsTest
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
