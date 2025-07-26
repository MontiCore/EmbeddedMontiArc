/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import jline.internal.Nullable;
import org.apache.commons.lang3.SystemUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

public class CNNArchSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements CNNArchInheritanceVisitor {

    private String compilationUnitPackage = "";

    private MathSymbolTableCreator mathSTC;
    private ArchitectureSymbol architecture;
    private String backend;
    private String customFilesPath = "";
    private String pythonPath = "";


    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
        initSuperSTC(resolvingConfig);
    }

    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final MutableScope enclosingScope,
                                     String customFilesPath,
                                     String pythonPath,
                                     String backend) {
        super(resolvingConfig, enclosingScope);
        setBackend(backend);
        setPythonPath(pythonPath);
        setCustomFilesPath(customFilesPath);
        initSuperSTC(resolvingConfig);
    }

    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        initSuperSTC(resolvingConfig);
    }

    public CNNArchSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                     final Deque<MutableScope> scopeStack,
                                     String customFilesPath,
                                     String pythonPath,
                                     String backend) {
        super(resolvingConfig, scopeStack);
        setBackend(backend);
        setPythonPath(pythonPath);
        setCustomFilesPath(customFilesPath);
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

    private void setBackend(String backend){
        this.backend = backend;
    }

    public String getBackend(){ return this.backend; }

    private void setCustomFilesPath(String customPythonFilesPath){
        this.customFilesPath = customPythonFilesPath;
    }

    public String getCustomFilesPath(){
        return this.customFilesPath;
    }

    private void setPythonPath (String pythonPath){ this.pythonPath = pythonPath; }

    public String getPythonPath (){ return this.pythonPath; }

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

        this.compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackageList());

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
        if(!getCustomFilesPath().equals("")) {
            createCustomPythonLayers();
            createCustomCPPLayers();
        }
    }

    public void endVisit(final ASTArchitecture node) {
        List<LayerVariableDeclarationSymbol> layerVariableDeclarations = new ArrayList<>();
        List<NetworkInstructionSymbol> networkInstructions = new ArrayList<>();

        for (ASTInstruction astInstruction : node.getInstructionsList()) {
            if (astInstruction.isPresentLayerVariableDeclaration()) {
                layerVariableDeclarations.add((LayerVariableDeclarationSymbol) astInstruction.getLayerVariableDeclaration().getSymbolOpt().get());
            }
            else if (astInstruction.isPresentNetworkInstruction()) {
                networkInstructions.add((NetworkInstructionSymbol) astInstruction.getNetworkInstruction().getSymbolOpt().get());
            }
        }

        architecture.setLayerVariableDeclarations(layerVariableDeclarations);
        architecture.setNetworkInstructions(networkInstructions);

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

    private void createCustomPythonLayers() {
        String[] pyFiles = null;
        File customFilesPath = new File(getCustomFilesPath() + "python/" + getBackend() + "/custom_layers/");
        if (customFilesPath.exists() && customFilesPath.isDirectory()) {
            pyFiles = customFilesPath.list();
        }

        if (pyFiles != null) {
            for (int index = 0; index < pyFiles.length; index++) {
                if (pyFiles[index].equals("__init__.py") || !(pyFiles[index].endsWith(".py"))) {
                    continue;
                }
                String nameWithoutExtension = pyFiles[index].substring(0, pyFiles[index].length() - 3);
                CustomPythonLayerDeclaration declaration = new CustomPythonLayerDeclaration(nameWithoutExtension, customFilesPath, pythonPath, "python");
                declaration.setParameters(declaration.extractParametersFromFile());
                addToScope(declaration);
            }
        }
    }

    private void createCustomCPPLayers() {
        String[] cppFiles = null;
        File customFilesPath = new File(getCustomFilesPath() + "cpp/custom_layers/");
        if (customFilesPath.exists() && customFilesPath.isDirectory()) {
            cppFiles = customFilesPath.list();
        }

        if (cppFiles != null) {
            for (int index = 0; index < cppFiles.length; index++) {
                if (!(cppFiles[index].endsWith(".cpp"))) {
                    continue;
                }
                String nameWithoutExtension = cppFiles[index].substring(0, cppFiles[index].length() - 4);
                CustomCPPLayerDeclaration declaration = new CustomCPPLayerDeclaration(nameWithoutExtension, customFilesPath, "cpp");
                declaration.setParameters(declaration.extractParametersFromFile());
                addToScope(declaration);
            }
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
//NEW
    @Override
    public void endVisit(ASTArchType node) {
        ArchTypeSymbol sym = (ArchTypeSymbol) node.getSymbolOpt().get();
        List<ASTArchSimpleExpression> astDimensions = node.getShape().getDimensionsList();
        Integer size = new Integer(astDimensions.size());
        java.lang.System.out.println("Size of astDimension is: " + size.toString());
        if (astDimensions.size() >= 1){
            sym.setChannelIndex(0);
        }
        if (astDimensions.size() >= 2){
            java.lang.System.out.println("Setting Height Index in CNNArchSymbolTable");
            sym.setHeightIndex(1);
        }
        if (astDimensions.size() >= 3){
            sym.setWidthIndex(2);
        }
        if (astDimensions.size() >= 4){
            java.lang.System.out.println("Setting Depth Index to 1 in CNNArchSymbolTable");
            sym.setDepthIndex(1);
            sym.setHeightIndex(2);
            sym.setWidthIndex(3);
        }
        List<ArchSimpleExpressionSymbol> dimensionList = new ArrayList<>(4);
        for (ASTArchSimpleExpression astExp : astDimensions){
            dimensionList.add((ArchSimpleExpressionSymbol) astExp.getSymbolOpt().get());
        }
        sym.setDimensionSymbols(dimensionList);
        sym.setDomain(node.getElementType());
    }
//END NEW
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
    public void visit(ASTTimeParameter ast) {
        ParameterSymbol variable = new ParameterSymbol(ast.getName());
        variable.setType(ParameterType.TIME_PARAMETER);
        addToScopeAndLinkWithNode(variable, ast);
    }

    @Override
    public void endVisit(ASTTimeParameter ast) {
        ParameterSymbol variable = (ParameterSymbol) ast.getSymbolOpt().get();
        if (ast.isPresentDefault()){
            variable.setDefaultExpression((ArchSimpleExpressionSymbol) ast.getDefault().getSymbolOpt().get());
        }
        else {
            ArchSimpleExpressionSymbol expression = new ArchSimpleExpressionSymbol();
            expression.setValue(1);
            variable.setDefaultExpression(expression);
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
    public void visit(ASTUnrollInstruction ast) {
        UnrollInstructionSymbol unrollInstruction = new UnrollInstructionSymbol(ast.getName());
        addToScopeAndLinkWithNode(unrollInstruction, ast);
    }

    @Override
    public void endVisit(ASTUnrollInstruction ast) {
        UnrollInstructionSymbol unrollInstruction = (UnrollInstructionSymbol) ast.getSymbolOpt().get();
        unrollInstruction.setBody((SerialCompositeElementSymbol) ast.getBody().getSymbolOpt().get());
        List<ArgumentSymbol> arguments = new ArrayList<>(6);
        
        for (ASTArchArgument astArgument : ast.getArgumentsList()){
            Optional<ArgumentSymbol> optArgument = astArgument.getSymbolOpt().map(e -> (ArgumentSymbol)e);
            optArgument.ifPresent(arguments::add);
        }
        unrollInstruction.setArguments(arguments);

        unrollInstruction.setTimeParameter((ParameterSymbol) ast.getTimeParameter().getSymbolOpt().get());

        removeCurrentScope();
    }

    @Override
    public void visit(ASTStreamInstruction ast) {
        StreamInstructionSymbol streamInstruction = new StreamInstructionSymbol();
        addToScopeAndLinkWithNode(streamInstruction, ast);
    }

    @Override
    public void endVisit(ASTStreamInstruction ast) {
        StreamInstructionSymbol streamInstruction = (StreamInstructionSymbol) ast.getSymbolOpt().get();
        streamInstruction.setBody((SerialCompositeElementSymbol) ast.getBody().getSymbolOpt().get());

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
