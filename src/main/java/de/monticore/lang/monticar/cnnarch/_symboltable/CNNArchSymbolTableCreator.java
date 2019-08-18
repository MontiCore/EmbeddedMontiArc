/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.monticar.cnnarch._ast.*;
import de.monticore.lang.monticar.cnnarch._visitor.CNNArchInheritanceVisitor;
import de.monticore.lang.monticar.cnnarch._visitor.CNNArchVisitor;
import de.monticore.lang.monticar.cnnarch._visitor.CNNArchDelegatorVisitor;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class CNNArchSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements CNNArchInheritanceVisitor {

    private String compilationUnitPackage = "";

    private MathSymbolTableCreator mathSTC;
    private ArchitectureSymbol architecture;


    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC(resolvingConfig);
    }

    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    private void initSuperSTC(final ResolvingConfiguration resolvingConfig) {
        this.mathSTC = new ModifiedMathSymbolTableCreator(resolvingConfig, scopeStack);
        CNNArchDelegatorVisitor visitor = new CNNArchDelegatorVisitor();
        visitor.setCNNArchVisitor(this);
        visitor.setMathVisitor(mathSTC);

        setRealThis(visitor);
    }

    /**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */
    public Scope createFromAST(de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchNode rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_650 Error by creating of the CNNArchSymbolTableCreatorTOP symbol table: top ast node is null");
        rootNode.accept(realThis);
        return getFirstCreatedScope();
    }

    private CNNArchVisitor realThis = this;

    @Override
    public CNNArchVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(CNNArchVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
        }
    }

    public MathSymbolTableCreator getMathSTC() {
        return mathSTC;
    }

    @Override
    public void visit(final ASTCNNArchCompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Script: " + compilationUnit.getName(),
                CNNArchSymbolTableCreator.class.getSimpleName());

        List<ImportStatement> imports = new ArrayList<>();

        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);

        putOnStack(artifactScope);

        CNNArchCompilationUnitSymbol compilationUnitSymbol = new CNNArchCompilationUnitSymbol(compilationUnit.getName());
        addToScopeAndLinkWithNode(compilationUnitSymbol, compilationUnit);
    }

    @Override
    public void endVisit(ASTCNNArchCompilationUnit ast) {
        CNNArchCompilationUnitSymbol compilationUnitSymbol = (CNNArchCompilationUnitSymbol) ast.getSymbolOpt().get();
        compilationUnitSymbol.setArchitecture((ArchitectureSymbol) ast.getArchitecture().getSymbolOpt().get());

        List<ParameterSymbol> parameters = new ArrayList<>(ast.getArchitectureParameterList().size());
        for (ASTArchitectureParameter astParameter : ast.getArchitectureParameterList()){
            parameters.add((ParameterSymbol) astParameter.getSymbolOpt().get());
        }
        compilationUnitSymbol.setParameters(parameters);

        List<IODeclarationSymbol> ioDeclarations = new ArrayList<>();
        for (ASTIODeclaration astIODeclaration : ast.getIoDeclarationsList()){
            ioDeclarations.add((IODeclarationSymbol) astIODeclaration.getSymbolOpt().get());
        }
        compilationUnitSymbol.setIoDeclarations(ioDeclarations);

        setEnclosingScopeOfNodes(ast);
    }

    public void visit(final ASTArchitecture node) {
        architecture = new ArchitectureSymbol();

        addToScopeAndLinkWithNode(architecture, node);

        createPredefinedConstants();
        createPredefinedLayers();
    }

    public void endVisit(final ASTArchitecture node) {
        List<LayerVariableDeclarationSymbol> layerVariableDeclarations = new ArrayList<>();
        List<SerialCompositeElementSymbol> streams = new ArrayList<>();
        List<UnrollSymbol> unrolls = new ArrayList<>();
        for (ASTInstruction astInstruction : node.getInstructionsList()){
            if (astInstruction.isPresentLayerVariableDeclaration()) {
                layerVariableDeclarations.add((LayerVariableDeclarationSymbol) astInstruction.getLayerVariableDeclaration().getSymbolOpt().get());
            }
            /*
            if(astInstruction instanceof ASTStream) {
                ASTStream astStream = (ASTStream) astInstruction;
                streams.add((SerialCompositeElementSymbol) astStream.getSymbolOpt().get());*/
            else if (astInstruction.isPresentStream()) {
                streams.add((SerialCompositeElementSymbol) astInstruction.getStream().getSymbolOpt().get());
            }else if(astInstruction.isPresentUnroll()) {
                unrolls.add((UnrollSymbol) astInstruction.getUnroll().getSymbolOpt().get());
                //System.err.println("Table 1: " + ((UnrollSymbol) astUnroll.getSymbolOpt().get()).getName());
                //System.err.println("Table 1_1: " + ((UnrollSymbol) astUnroll.getSymbolOpt().get()).getBody().getElements().toString());
            }
        }


        architecture.setLayerVariableDeclarations(layerVariableDeclarations);
        architecture.setStreams(streams);
        architecture.setUnrolls(unrolls);

        removeCurrentScope();
    }

    private void createPredefinedConstants(){
        addToScope(AllPredefinedVariables.createTrueConstant());
        addToScope(AllPredefinedVariables.createFalseConstant());
    }

    private void createPredefinedLayers(){
        for (LayerDeclarationSymbol sym : AllPredefinedLayers.createList()){
            addToScope(sym);
        }
        for (UnrollDeclarationSymbol sym : AllPredefinedLayers.createUnrollList()){
            addToScope(sym);
        }
    }

    @Override
    public void endVisit(ASTArchitectureParameter node) {
        ParameterSymbol variable = new ParameterSymbol(node.getName());
        variable.setType(ParameterType.ARCHITECTURE_PARAMETER);
        if (node.isPresentDefault()){
            variable.setDefaultExpression((ArchSimpleExpressionSymbol) node.getDefault().getSymbolOpt().get());
        }

        addToScopeAndLinkWithNode(variable, node);
    }

    @Override
    public void visit(ASTIODeclaration ast) {
        IODeclarationSymbol iODeclaration = new IODeclarationSymbol(ast.getName());
        addToScopeAndLinkWithNode(iODeclaration, ast);
    }

    @Override
    public void endVisit(ASTIODeclaration ast) {
        IODeclarationSymbol iODeclaration = (IODeclarationSymbol) ast.getSymbolOpt().get();
        if (ast.isPresentArrayDeclaration()){
            iODeclaration.setArrayLength(ast.getArrayDeclaration().getIntLiteral().getNumber().get().intValue());
        }
        iODeclaration.setInput(ast.isPresentIn());
        iODeclaration.setType((ArchTypeSymbol) ast.getType().getSymbolOpt().get());
    }

    @Override
    public void visit(ASTArchType ast) {
        ArchTypeSymbol sym = new ArchTypeSymbol();
        addToScopeAndLinkWithNode(sym, ast);
    }

    @Override
    public void endVisit(ASTArchType node) {
        ArchTypeSymbol sym = (ArchTypeSymbol) node.getSymbolOpt().get();
        List<ASTArchSimpleExpression> astDimensions = node.getShape().getDimensionsList();

        if (astDimensions.size() >= 1){
            sym.setChannelIndex(0);
        }
        if (astDimensions.size() >= 2){
            sym.setHeightIndex(1);
        }
        if (astDimensions.size() >= 3){
            sym.setWidthIndex(2);
        }
        List<ArchSimpleExpressionSymbol> dimensionList = new ArrayList<>(3);
        for (ASTArchSimpleExpression astExp : astDimensions){
            dimensionList.add((ArchSimpleExpressionSymbol) astExp.getSymbolOpt().get());
        }
        sym.setDimensionSymbols(dimensionList);
        sym.setDomain(node.getElementType());
    }

    @Override
    public void visit(ASTLayerDeclaration ast) {
        LayerDeclarationSymbol layerDeclaration = new LayerDeclarationSymbol(ast.getName());
        addToScopeAndLinkWithNode(layerDeclaration, ast);
    }

    @Override
    public void endVisit(ASTLayerDeclaration ast) {
        LayerDeclarationSymbol layerDeclaration = (LayerDeclarationSymbol) ast.getSymbolOpt().get();
        layerDeclaration.setBody((SerialCompositeElementSymbol) ast.getBody().getSymbolOpt().get());

        List<ParameterSymbol> parameters = new ArrayList<>(4);
        for (ASTLayerParameter astParam : ast.getParametersList()){
            ParameterSymbol parameter = (ParameterSymbol) astParam.getSymbolOpt().get();
            parameters.add(parameter);
        }
        layerDeclaration.setParameters(parameters);

        removeCurrentScope();
    }

    @Override
    public void visit(ASTLayerParameter ast) {
        ParameterSymbol variable = new ParameterSymbol(ast.getName());
        variable.setType(ParameterType.LAYER_PARAMETER);
        addToScopeAndLinkWithNode(variable, ast);
    }

    @Override
    public void endVisit(ASTLayerParameter ast) {
        ParameterSymbol variable = (ParameterSymbol) ast.getSymbolOpt().get();
        if (ast.isPresentDefault()){
            variable.setDefaultExpression((ArchSimpleExpressionSymbol) ast.getDefault().getSymbolOpt().get());
        }
    }

    @Override
    public void endVisit(ASTArchSimpleExpression ast) {
        ArchSimpleExpressionSymbol sym = new ArchSimpleExpressionSymbol();
        MathExpressionSymbol mathExp = null;
        if (ast.isPresentArithmeticExpression())
            mathExp = (MathExpressionSymbol) ast.getArithmeticExpression().getSymbolOpt().get();
        else if (ast.isPresentBooleanExpression())
            mathExp = (MathExpressionSymbol) ast.getBooleanExpression().getSymbolOpt().get();
        else if (ast.isPresentTupleExpression())
            mathExp = (MathExpressionSymbol) ast.getTupleExpression().getSymbolOpt().get();
        else
            sym.setValue(ast.getString().getValue());
        sym.setMathExpression(mathExp);
        addToScopeAndLinkWithNode(sym, ast);
    }

    @Override
    public void endVisit(ASTArchExpression node) {
        if (node.isPresentExpression()){
            addToScopeAndLinkWithNode(node.getExpression().getSymbolOpt().get(), node);
        }
        else {
            addToScopeAndLinkWithNode(node.getSequence().getSymbolOpt().get(), node);
        }
    }

    @Override
    public void visit(ASTArchValueRange node) {
        ArchRangeExpressionSymbol sym = new ArchRangeExpressionSymbol();
        addToScopeAndLinkWithNode(sym, node);
    }

    @Override
    public void endVisit(ASTArchValueRange node) {
        ArchRangeExpressionSymbol sym = (ArchRangeExpressionSymbol) node.getSymbolOpt().get();
        sym.setParallel(node.isPresentParallel());
        sym.setStartSymbol((ArchSimpleExpressionSymbol) node.getStart().getSymbolOpt().get());
        sym.setEndSymbol((ArchSimpleExpressionSymbol) node.getEnd().getSymbolOpt().get());
    }

    @Override
    public void visit(ASTArchParallelSequence node) {
        ArchSequenceExpressionSymbol sym = new ArchSequenceExpressionSymbol();
        addToScopeAndLinkWithNode(sym, node);
    }

    @Override
    public void endVisit(ASTArchParallelSequence node) {
        ArchSequenceExpressionSymbol sym = (ArchSequenceExpressionSymbol) node.getSymbolOpt().get();

        List<List<ArchSimpleExpressionSymbol>> elements = new ArrayList<>();
        for (ASTArchSerialSequence serialSequenceAST : node.getParallelValuesList()) {
            List<ArchSimpleExpressionSymbol> serialElements = new ArrayList<>();
            for (ASTArchSimpleExpression astExpression : serialSequenceAST.getSerialValuesList()) {
                serialElements.add((ArchSimpleExpressionSymbol) astExpression.getSymbolOpt().get());
            }
            elements.add(serialElements);
        }
        sym.setElements(elements);
    }

    @Override
    public void visit(ASTUnroll ast) {
        UnrollSymbol layer = new UnrollSymbol(ast.getName());
        addToScopeAndLinkWithNode(layer, ast);
    }

    @Override
    public void endVisit(ASTUnroll ast) {
        UnrollSymbol layer = (UnrollSymbol) ast.getSymbolOpt().get();
        SerialCompositeElementSymbol sces = new SerialCompositeElementSymbol();
        List<ArchitectureElementSymbol> elements = new ArrayList<>();
        for (ASTArchitectureElement astElement : ast.getBody().getElementsList()){
            elements.add((ArchitectureElementSymbol) astElement.getSymbolOpt().get());
        }
        sces.setElements(elements);
        layer.setBody(sces);
        //layer.getDeclaration().setBody(sces);

        //layer.setElements(elements);

        List<ArgumentSymbol> arguments = new ArrayList<>(6);
        for (ASTArchArgument astArgument : ast.getArgumentsList()){
            Optional<ArgumentSymbol> optArgument = astArgument.getSymbolOpt().map(e -> (ArgumentSymbol)e);
            optArgument.ifPresent(arguments::add);
        }
        layer.setArguments(arguments);





        /*List<ArchitectureElementSymbol> elements = new ArrayList<>();
        for (ASTStream astStream : ast.getGroupsList()){
            elements.add((SerialCompositeElementSymbol) astStream.getSymbolOpt().get());
        }
        compositeElement.setElements(elements);
        */

        removeCurrentScope();
    }

    @Override
    public void visit(ASTParallelBlock node) {
        ParallelCompositeElementSymbol compositeElement = new ParallelCompositeElementSymbol();
        addToScopeAndLinkWithNode(compositeElement, node);
    }

    @Override
    public void endVisit(ASTParallelBlock node) {
        ParallelCompositeElementSymbol compositeElement = (ParallelCompositeElementSymbol) node.getSymbolOpt().get();

        List<ArchitectureElementSymbol> elements = new ArrayList<>();
        for (ASTStream astStream : node.getGroupsList()){
            elements.add((SerialCompositeElementSymbol) astStream.getSymbolOpt().get());
        }
        compositeElement.setElements(elements);

        removeCurrentScope();
    }

    @Override
    public void visit(ASTLayerVariableDeclaration ast) {
        LayerVariableDeclarationSymbol layerVariableDeclaration = new LayerVariableDeclarationSymbol(ast.getName());
        addToScopeAndLinkWithNode(layerVariableDeclaration, ast);
    }

    @Override
    public void endVisit(ASTLayerVariableDeclaration ast) {
        LayerVariableDeclarationSymbol layerVariableDeclaration = (LayerVariableDeclarationSymbol) ast.getSymbolOpt().get();
        layerVariableDeclaration.setLayer((LayerSymbol) ast.getLayer().getSymbolOpt().get());
    }

    @Override
    public void visit(ASTStream ast) {
        SerialCompositeElementSymbol compositeElement = new SerialCompositeElementSymbol();
        addToScopeAndLinkWithNode(compositeElement, ast);
    }

    @Override
    public void endVisit(ASTStream ast) {
        SerialCompositeElementSymbol compositeElement = (SerialCompositeElementSymbol) ast.getSymbolOpt().get();
        List<ArchitectureElementSymbol> elements = new ArrayList<>();
        for (ASTArchitectureElement astElement : ast.getElementsList()){
            elements.add((ArchitectureElementSymbol) astElement.getSymbolOpt().get());
        }
        compositeElement.setElements(elements);

        removeCurrentScope();
    }

    @Override
    public void visit(ASTLayer ast) {
        LayerSymbol layer = new LayerSymbol(ast.getName());
        addToScopeAndLinkWithNode(layer, ast);
    }

    @Override
    public void endVisit(ASTLayer ast) {
        LayerSymbol layer = (LayerSymbol) ast.getSymbolOpt().get();

        List<ArgumentSymbol> arguments = new ArrayList<>(6);
        for (ASTArchArgument astArgument : ast.getArgumentsList()){
            Optional<ArgumentSymbol> optArgument = astArgument.getSymbolOpt().map(e -> (ArgumentSymbol)e);
            optArgument.ifPresent(arguments::add);
        }
        layer.setArguments(arguments);

        removeCurrentScope();
    }

    @Override
    public void endVisit(ASTArchArgument node) {
        ArchExpressionSymbol value;
        value = (ArchExpressionSymbol) node.getRhs().getSymbolOpt().get();

        ArgumentSymbol argument = new ArgumentSymbol(node.getName());
        argument.setRhs(value);
        addToScopeAndLinkWithNode(argument, node);
    }

    public void visit(ASTConstant node) {
        ConstantSymbol constant = new ConstantSymbol();
        addToScopeAndLinkWithNode(constant, node);
    }

    public void endVisit(ASTConstant node) {
        ConstantSymbol constant = (ConstantSymbol) node.getSymbolOpt().get();
        constant.setExpression((ArchSimpleExpressionSymbol) node.getArchSimpleExpression().getSymbolOpt().get());
        removeCurrentScope();
    }

    public void visit(ASTVariable node) {
        VariableSymbol variableSymbol = new VariableSymbol(node.getName());
        addToScopeAndLinkWithNode(variableSymbol, node);
    }

    @Override
    public void endVisit(ASTVariable node) {
        VariableSymbol variableSymbol = (VariableSymbol) node.getSymbolOpt().get();

        if (node.isPresentMember()) {
            variableSymbol.setMember(node.getMember());
        }

        if (node.isPresentIndex()) {
            variableSymbol.setArrayAccess((ArchSimpleExpressionSymbol) node.getIndex().getSymbolOpt().get());
        }

        removeCurrentScope();
    }

    @Override
    public void visit(ASTArrayAccessLayer node) {
        LayerSymbol layer = new LayerSymbol(AllPredefinedLayers.GET_NAME);
        addToScopeAndLinkWithNode(layer, node);
    }

    @Override
    public void endVisit(ASTArrayAccessLayer node) {
        LayerSymbol layer = (LayerSymbol) node.getSymbolOpt().get();
        ArgumentSymbol indexArgument = new ArgumentSymbol.Builder()
                .parameter(layer.getDeclaration().getParameter(AllPredefinedLayers.INDEX_NAME).get())
                .value((ArchSimpleExpressionSymbol) node.getIndex().getSymbolOpt().get())
                .build();
        indexArgument.setAstNode(node.getIndex());
        addToScope(indexArgument);
        layer.setArguments(Collections.singletonList(indexArgument));

        removeCurrentScope();
    }

    @Override
    public void endVisit(ASTTupleExpression node) {
        TupleExpressionSymbol symbol = new TupleExpressionSymbol();

        for (ASTArchArithmeticExpression expression : node.getExpressionsList()){
            if (expression.getSymbolOpt().isPresent()) {
                symbol.add((MathExpressionSymbol) expression.getSymbolOpt().get());
            }
        }

        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void endVisit(ASTArchSimpleArithmeticExpression node) {
        MathExpressionSymbol sym = null;
        if (node.isPresentNumberExpression())
            sym = (MathExpressionSymbol) node.getNumberExpression().getSymbolOpt().get();
        else if (node.isPresentNameExpression())
            sym = (MathExpressionSymbol) node.getNameExpression().getSymbolOpt().get();
        else if (node.isPresentMathDottedNameExpression())
            sym = (MathExpressionSymbol) node.getMathDottedNameExpression().getSymbolOpt().get();
        else if (node.isPresentMathAssignmentDeclarationStatement())
            sym = (MathExpressionSymbol) node.getMathAssignmentDeclarationStatement().getSymbolOpt().get();
        else if (node.isPresentMathAssignmentStatement())
            sym = (MathExpressionSymbol) node.getMathAssignmentStatement().getSymbolOpt().get();

        addToScopeAndLinkWithNode(sym, node);
    }

    @Override
    public void endVisit(ASTArchComplexArithmeticExpression node) {
        MathArithmeticExpressionSymbol arithmSym = new MathArithmeticExpressionSymbol();
        if (node.getLeftExpression().getSymbolOpt().isPresent()) {
            arithmSym.setLeftExpression((MathExpressionSymbol) node.getLeftExpression().getSymbolOpt().get());
        }
        if (node.getRightExpression().getSymbolOpt().isPresent()) {
            arithmSym.setRightExpression((MathExpressionSymbol) node.getRightExpression().getSymbolOpt().get());
        }
        arithmSym.setOperator(node.getOperator());

        addToScopeAndLinkWithNode(arithmSym, node);
    }

    @Override
    public void endVisit(ASTArchSimpleBooleanExpression node) {
        MathExpressionSymbol sym = null;
        if (node.isPresentBooleanExpression())
            sym = new MathBooleanExpressionSymbol(node.getBooleanExpression().getBooleanLiteral().getValue());
        else if (node.isPresentBooleanNotExpression())
            sym = (MathExpressionSymbol) node.getBooleanNotExpression().getSymbolOpt().get();
        else if (node.isPresentLogicalNotExpression())
            sym = (MathExpressionSymbol) node.getLogicalNotExpression().getSymbolOpt().get();

        addToScopeAndLinkWithNode(sym, node);
    }

    @Override
    public void endVisit(ASTArchComplexBooleanExpression node) {
        MathArithmeticExpressionSymbol arithmSym = new MathArithmeticExpressionSymbol();
        if (node.getLeftExpression().getSymbolOpt().isPresent()) {
            arithmSym.setLeftExpression((MathExpressionSymbol) node.getLeftExpression().getSymbolOpt().get());
        }
        if (node.getRightExpression().getSymbolOpt().isPresent()) {
            arithmSym.setRightExpression((MathExpressionSymbol) node.getRightExpression().getSymbolOpt().get());
        }
        arithmSym.setOperator(node.getOperator());

        addToScopeAndLinkWithNode(arithmSym, node);
    }

    @Override
    public void endVisit(ASTArchBracketExpression node) {
        MathExpressionSymbol sym = (MathExpressionSymbol) node.getArchMathExpression().getSymbolOpt().get();
        addToScopeAndLinkWithNode(sym, node);
    }

    @Override
    public void endVisit(ASTArchPreMinusExpression node) {
        MathPreOperatorExpressionSymbol symbol = new MathPreOperatorExpressionSymbol();
        symbol.setMathExpressionSymbol((MathExpressionSymbol) node.getArchMathExpression().getSymbolOpt().get());
        symbol.setOperator("-");
        addToScopeAndLinkWithNode(symbol, node);
    }
}