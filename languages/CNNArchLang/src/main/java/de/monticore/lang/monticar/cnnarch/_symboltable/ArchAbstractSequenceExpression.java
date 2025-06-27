/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

abstract public class ArchAbstractSequenceExpression extends ArchExpressionSymbol {


    public ArchAbstractSequenceExpression() {
        super();
    }

    abstract public Optional<List<List<ArchSimpleExpressionSymbol>>> getElements();

    abstract public boolean isParallelSequence();

    abstract public boolean isSerialSequence();

    @Override
    public boolean isSequence(){
        return true;
    }

    @Override
    public Optional<Object> getValue() {
        if (isResolved()){
            List<List<Object>> parallelValues = new ArrayList<>(getParallelLength().get());
            for (List<ArchSimpleExpressionSymbol> serialElements : getElements().get()){
                List<Object> serialValues = new ArrayList<>();
                for (ArchSimpleExpressionSymbol element : serialElements){
                    serialValues.add(element.getValue().get());
                }
                parallelValues.add(serialValues);
            }
            return Optional.of(parallelValues);
        }
        else{
            return Optional.empty();
        }
    }

    @Override
    public boolean isBoolean(){
        return false;
    }

    @Override
    public boolean isNumber(){
        return false;
    }

    @Override
    public boolean isTuple(){
        return false;
    }
}
