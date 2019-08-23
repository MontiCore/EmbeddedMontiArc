/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.instruction;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;

import java.util.Optional;

public class ExecuteDynamicConnects implements Instruction {

    protected Optional<String> beforeComponent;

    public ExecuteDynamicConnects(){
        beforeComponent = Optional.empty();
    }

    public ExecuteDynamicConnects(String after){
        beforeComponent = Optional.of(after);
    }

    public ExecuteDynamicConnects(Optional<String> after){
        beforeComponent = after;
    }

    @Override
    public String getTargetLanguageInstruction() {

        if(!beforeComponent.isPresent()){
            return  "executeDynamicConnects(NULL);\n";
        }
        return  "executeDynamicConnects(&"+
                GeneralHelperMethods.getTargetLanguageVariableInstanceName(beforeComponent.get())+");\n";
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return true;
    }

    @Override
    public boolean isExecuteInstruction() {
        return false;
    }

    public Optional<String> getBeforeComponent() {
        return beforeComponent;
    }

    public String getBeforeComponentName(){
        return beforeComponent.orElse("");
    }
}
