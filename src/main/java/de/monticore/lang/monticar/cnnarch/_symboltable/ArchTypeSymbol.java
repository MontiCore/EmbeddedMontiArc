/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.*;


public class ArchTypeSymbol extends CommonSymbol {

    public static final ArchTypeKind KIND = new ArchTypeKind();

    protected static final String DEFAULT_ELEMENT_TYPE = "Q(-oo:oo)";
    private ASTElementType domain;

    private int channelIndex = -1;
    private int heightIndex = -1;
    private int widthIndex = -1;
    private int depthIndex = -1; 
    private List<ArchSimpleExpressionSymbol> dimensions = new ArrayList<>();

    public ArchTypeSymbol() {
        super("", KIND);
        ASTElementType elementType = new ASTElementType();
        elementType.setName(DEFAULT_ELEMENT_TYPE);
        setDomain(elementType);
    }

    public ASTElementType getDomain() {
        return domain;
    }

    public void setDomain(ASTElementType domain) {
        this.domain = domain;
    }

    public int getHeightIndex() {
        return heightIndex;
    }

    public HashMap<String, String> getElementRange() {
        String min="", max="";
        Optional<ASTRange> range = domain.getRangeOpt();
        if (range.isPresent() && (domain.isRational() || domain.isWholeNumber() || domain.isNaturalNumber() || domain.isComplex())) {
            if(range.get().hasNoLowerLimit()) { min="-inf"; } else {
                min = Double.toString(range.get().getStartValue().doubleValue());
            }
            if (range.get().hasNoUpperLimit()) { max="inf"; } else {
                max = Double.toString(range.get().getEndValue().doubleValue());
            }
        } else { // domain.isBoolean()
            min = "0";
            max = "1";
        }
        HashMap result = new HashMap<String, String>();
        result.put("min", min);
        result.put("max", max);
        return result;
    }

    public void setHeightIndex(int heightIndex) {
        this.heightIndex = heightIndex;
    }

    public int getWidthIndex() {
        return widthIndex;
    }

    public void setWidthIndex(int widthIndex) {
        this.widthIndex = widthIndex;
    }

    public int getChannelIndex() {
        return channelIndex;
    }

    public void setChannelIndex(int channelIndex) {
        this.channelIndex = channelIndex;
    }
//NEW
    public int getDepthIndex(){
        return depthIndex;
    }
    
    public void setDepthIndex(int depthIndex){
        this.depthIndex = depthIndex;
    }
//END NEW
    public ArchSimpleExpressionSymbol getHeightSymbol() {
        if (getHeightIndex() == -1){
            return ArchSimpleExpressionSymbol.of(1);
        }
        return getDimensionSymbols().get(getHeightIndex());
    }

    public ArchSimpleExpressionSymbol getWidthSymbol() {
        if (getWidthIndex() == -1){
            return ArchSimpleExpressionSymbol.of(1);
        }
        return getDimensionSymbols().get(getWidthIndex());
    }

    public ArchSimpleExpressionSymbol getChannelsSymbol() {
        if (getChannelIndex() == -1){
            return ArchSimpleExpressionSymbol.of(1);
        }
        return getDimensionSymbols().get(getChannelIndex());
    }

//NEW
    public ArchSimpleExpressionSymbol getDepthSymbol() {
        if (getDepthIndex() == -1){
            return ArchSimpleExpressionSymbol.of(1);
        }
        return getDimensionSymbols().get(getDepthIndex());
    }

    public Integer getDepth(){
        return getDepthSymbol().getIntValue().get();
    }
//END NEW

    public Integer getWidth(){
        return getWidthSymbol().getIntValue().get();
    }

    public Integer getHeight(){
        return getHeightSymbol().getIntValue().get();
    }

    public Integer getChannels(){
        return getChannelsSymbol().getIntValue().get();
    }

    public void setDimensionSymbols(List<ArchSimpleExpressionSymbol> dimensions) {
        this.dimensions = dimensions;
    }

    public List<ArchSimpleExpressionSymbol> getDimensionSymbols() {
        return dimensions;
    }

    public void setDimensions(List<Integer> dimensionList){
        List<ArchSimpleExpressionSymbol> symbolList = new ArrayList<>(dimensionList.size());
        for (int e : dimensionList){
            symbolList.add(ArchSimpleExpressionSymbol.of(e));
        }
        setDimensionSymbols(symbolList);
    }

    public List<Integer> getDimensions(){
        List<Integer> dimensionList = new ArrayList<>();
        for (ArchSimpleExpressionSymbol exp : getDimensionSymbols()){
            dimensionList.add(exp.getIntValue().get());
        }
        return dimensionList;
    }
    
    public Set<ParameterSymbol> resolve() {
        if (!isResolved()){
            if (isResolvable()){
                for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
                    dimension.resolveOrError();
                }
            }
        }
        return getUnresolvableParameters();
    }

    public boolean isResolvable(){
        boolean isResolvable = true;
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            if (!dimension.isResolvable()){
                isResolvable = false;
            }
        }
        return isResolvable;
    }

    public boolean isResolved(){
        boolean isResolved = true;
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            if (!dimension.isResolved()){
                isResolved = false;
            }
        }
        return isResolved;
    }

    public Set<ParameterSymbol> getUnresolvableParameters(){
        Set<ParameterSymbol> unresolvableParameters = new HashSet<>();
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            unresolvableParameters.addAll(dimension.getUnresolvableParameters());
        }
        return unresolvableParameters;
    }

    public void checkIfResolvable(Set<ParameterSymbol> seenVariables) {
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            dimension.checkIfResolvable(seenVariables);
        }
    }

    @Override
    public void setEnclosingScope(MutableScope scope) {
        super.setEnclosingScope(scope);
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            dimension.putInScope(scope);
        }
    }

    public void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);
            for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
                dimension.putInScope(scope);
            }
        }
    }

    public ArchTypeSymbol preResolveDeepCopy() {
        ArchTypeSymbol copy = new ArchTypeSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        copy.setDomain(getDomain());
        copy.setWidthIndex(getWidthIndex());
        copy.setChannelIndex(getChannelIndex());
        copy.setHeightIndex(getHeightIndex());
        copy.setDepthIndex(getDepthIndex());
        List<ArchSimpleExpressionSymbol> dimensionCopies = new ArrayList<>();
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            dimensionCopies.add(dimension.preResolveDeepCopy());
        }
        copy.setDimensionSymbols(dimensionCopies);

        return copy;
    }

    public void printDimensions(){
        for (ArchSimpleExpressionSymbol dimension : getDimensionSymbols()){
            //System.out.println("From Dimension list: " + dimension.getTextualRepresentation());
        }
    }

    public static class Builder{
        private int height = 1;
        private int width = 1;
        private int channels = 1;
        private int depth = 0;
        private ASTElementType domain = null;

        public Builder height(int height){
            this.height = height;
            return this;
        }
        public Builder width(int width){
            this.width = width;
            return this;
        }
        public Builder channels(int channels){
            this.channels = channels;
            return this;
        }
        public Builder depth(int depth){
            this.depth = depth;
            return this;
        }
        public Builder elementType(ASTElementType domain){
            this.domain = domain;
            return this;
        }
        public Builder elementType(String start, String end){
            domain = new ASTElementType();
            domain.setName("Q"); //("Q(" + start + ":" + end +")");
            ASTRange range = new ASTRange();
            range.setStartValue(start);
            range.setEndValue(end);
            domain.setRange(range);
            return this;
        }
        public Builder elementType(String name, String start, String end){
            domain = new ASTElementType();
            domain.setName(name); //("Q(" + start + ":" + end +")");
            ASTRange range = new ASTRange();
            range.setStartValue(start);
            range.setEndValue(end);
            domain.setRange(range);
            return this;
        }

        public ArchTypeSymbol build(){
            ArchTypeSymbol sym = new ArchTypeSymbol();
            sym.setChannelIndex(0);
            sym.setHeightIndex(1);
            sym.setWidthIndex(2);
            
            if (this.depth != 0){
                sym.setHeightIndex(2);
                sym.setWidthIndex(3);
                sym.setDepthIndex(1);
                sym.setDimensions(Arrays.asList(channels, depth, height, width)); //Dimensions are in this order for mxnet
                //sym.printDimensions();
            }

            else {
                sym.setDimensions(Arrays.asList(channels, height, width));
            }

            if (domain == null){
                domain = new ASTElementType();
                domain.setName(DEFAULT_ELEMENT_TYPE);
            }
            sym.setDomain(domain);
            return sym;
        }
        
    }
}
