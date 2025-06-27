/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.Scope;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class ArchRangeExpressionSymbol extends ArchAbstractSequenceExpression {

    private ArchSimpleExpressionSymbol startSymbol;
    private ArchSimpleExpressionSymbol endSymbol;
    private boolean parallel;
    private List<List<ArchSimpleExpressionSymbol>> elements = null;


    protected ArchRangeExpressionSymbol() {
        super();
    }

    public ArchSimpleExpressionSymbol getStartSymbol() {
        return startSymbol;
    }

    protected void setStartSymbol(ArchSimpleExpressionSymbol startSymbol) {
        this.startSymbol = startSymbol;
    }

    public ArchSimpleExpressionSymbol getEndSymbol() {
        return endSymbol;
    }

    protected void setEndSymbol(ArchSimpleExpressionSymbol endSymbol) {
        this.endSymbol = endSymbol;
    }

    public boolean isParallel() {
        return parallel;
    }

    public void setParallel(boolean parallel) {
        this.parallel = parallel;
    }

    @Override
    public void reset() {
        getStartSymbol().reset();
        getEndSymbol().reset();
        setUnresolvableParameters(null);
    }

    @Override
    public boolean isParallelSequence() {
        return isParallel();
    }

    @Override
    public boolean isSerialSequence() {
        return !isParallel();
    }

    /*private Optional<Integer> getLength(){
        Optional<Integer> optLength = Optional.empty();
        if (isResolved()){
            Object startValue = getEndSymbol().getValue().get();
            Object endValue = getEndSymbol().getValue().get();
            if (startValue instanceof Integer && endValue instanceof Integer) {
                int start = (Integer) startValue;
                int end = (Integer) endValue;
                optLength = Optional.of(Math.abs(end - start) + 1);
            }
        }
        return optLength;
    }*/

    @Override
    public Set<ParameterSymbol> resolve() {
        if (!isResolved()){
            if (isResolvable()){

                getStartSymbol().resolveOrError();
                getEndSymbol().resolveOrError();
            }
        }
        return getUnresolvableParameters();
    }

    @Override
    public boolean isResolved() {
        return getStartSymbol().isResolved() && getEndSymbol().isResolved();
    }

    @Override
    public Optional<List<List<ArchSimpleExpressionSymbol>>> getElements() {
        if (elements == null){
            if (isResolved()){
                int start = startSymbol.getIntValue().get();
                int end = endSymbol.getIntValue().get();
                List<Integer> range;
                range = IntStream.rangeClosed(start, end).boxed().collect(Collectors.toList());

                List<List<ArchSimpleExpressionSymbol>> elementList = new ArrayList<>();
                if (isParallel()){
                    for (int element : range){
                        List<ArchSimpleExpressionSymbol> values = new ArrayList<>(1);
                        values.add(ArchSimpleExpressionSymbol.of(element));
                        elementList.add(values);
                    }
                }
                else {
                    List<ArchSimpleExpressionSymbol> values = new ArrayList<>();
                    for (int element : range){
                        values.add(ArchSimpleExpressionSymbol.of(element));
                    }
                    elementList.add(values);
                }

                this.elements = elementList;
            }
        }
        return Optional.ofNullable(elements);
    }

    @Override
    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        getStartSymbol().checkIfResolvable(allParameters);
        unresolvableParameters.addAll(getStartSymbol().getUnresolvableParameters());
        getEndSymbol().checkIfResolvable(allParameters);
        unresolvableParameters.addAll(getEndSymbol().getUnresolvableParameters());
    }

    @Override
    public String getTextualRepresentation() {
        String separator = isParallel() ? "|" : "->";
        return getStartSymbol().getTextualRepresentation() + separator + ".." + separator + getEndSymbol().getTextualRepresentation();
    }

    @Override
    protected void putInScope(Scope scope) {
        super.putInScope(scope);
        getStartSymbol().putInScope(scope);
        getEndSymbol().putInScope(scope);
    }

    @Override
    public ArchRangeExpressionSymbol preResolveDeepCopy(){
        ArchRangeExpressionSymbol copy = new ArchRangeExpressionSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        copy.setParallel(isParallel());
        copy.setStartSymbol(getStartSymbol().preResolveDeepCopy());
        copy.setEndSymbol(getEndSymbol().preResolveDeepCopy());
        return copy;
    }

    public static ArchRangeExpressionSymbol of(ArchSimpleExpressionSymbol start, ArchSimpleExpressionSymbol end, boolean parallel){
        ArchRangeExpressionSymbol sym = new ArchRangeExpressionSymbol();
        sym.setStartSymbol(start);
        sym.setEndSymbol(end);
        sym.setParallel(parallel);
        return sym;
    }
}
