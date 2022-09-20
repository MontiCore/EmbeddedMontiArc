/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._symboltable.EmbeddedMontiArcBehaviorSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._visitor.EmbeddedMontiArcBehaviorVisitor;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreatorTOP;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure.ModifiedEMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchSymbolTableCreator;
import de.monticore.lang.monticar.emadl._ast.ASTBehaviorEmbedding;
import de.monticore.lang.monticar.emadl._visitor.EMADLDelegatorVisitor;
import de.monticore.lang.monticar.emadl._visitor.EMADLVisitor;
import de.monticore.lang.monticar.emadl._visitor.ModularEMADLDelegatorVisitor;
import de.monticore.lang.monticar.emadl._visitor.ModularNetworkVisitor;
import de.monticore.lang.monticar.emadl.modularcnn.ArchitectureNode;
import de.monticore.lang.monticar.emadl.modularcnn.ModularCNNSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class EMADLSymbolTableCreator extends de.monticore.symboltable.CommonSymbolTableCreator
        implements EMADLVisitor {
    
    private final ModularEMADLDelegatorVisitor visitor = new ModularEMADLDelegatorVisitor();

    private EmbeddedMontiArcMathSymbolTableCreatorTOP emamSTC;
    private CNNArchSymbolTableCreator cnnArchSTC;
    private MathOptSymbolTableCreator mathOptSTC;
    private EmbeddedMontiArcDynamicSymbolTableCreator emadSTC;
    private EmbeddedMontiArcBehaviorVisitor emaBehaviorSTC;
    private ModularNetworkVisitor mcnnSTC;

    private ArrayList<ArchitectureNode> archNodes = null;


    public EMADLSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, enclosingScope);
        this.archNodes = archNodes;
        initSuperSTC(resolvingConfig);
    }

    public EMADLSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope,
            String customFilesPath, String pythonPath, String backend, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, enclosingScope);
        this.archNodes = archNodes;
        initSuperSTC(resolvingConfig, customFilesPath, pythonPath, backend);
    }

    public EMADLSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, scopeStack);
        this.archNodes = archNodes;
        initSuperSTC(resolvingConfig);
    }

    public EMADLSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack,
                                   String customFilesPath, String pythonPath, String backend, ArrayList<ArchitectureNode> archNodes) {
        super(resolvingConfig, scopeStack);
        this.archNodes = archNodes;
        initSuperSTC(resolvingConfig, customFilesPath, pythonPath, backend);
    }

    private void initSuperSTC(final ResolvingConfiguration resolvingConfig) {
        Log.info("INIT_SUPER_STC","SSTC_INIT_1");
        this.cnnArchSTC = new CNNArchSymbolTableCreator(resolvingConfig, scopeStack);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);
        this.mathOptSTC = new MathOptSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC = new ModifiedEMADynamicSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC.setInstanceSymbolCreator(new ModifiedEMAComponentInstanceSymbolCreator()); //Use an instance symbo, creator that adds math statement to instances
        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack);

        this.mcnnSTC = new ModularCNNSymbolTableCreator(resolvingConfig, scopeStack, archNodes);

        visitor.setEMADLVisitor(this);
        visitor.setCNNArchVisitor(cnnArchSTC);

        visitor.setEmbeddedMontiArcMathVisitor(emamSTC);
        visitor.setEmbeddedMontiArcVisitor(emadSTC);
        visitor.setEmbeddedMontiArcDynamicVisitor(emadSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(emaBehaviorSTC);
        visitor.setMathVisitor(mathOptSTC);
        visitor.setExpressionsBasisVisitor(mathOptSTC);
        visitor.setCommonExpressionsVisitor(mathOptSTC);
        visitor.setAssignmentExpressionsVisitor(mathOptSTC);
        visitor.setMatrixExpressionsVisitor(mathOptSTC);
        visitor.setMatrixVisitor(mathOptSTC);
        visitor.setTypes2Visitor(mathOptSTC);
        visitor.setMathOptVisitor(mathOptSTC);

        visitor.setCommon2Visitor(emamSTC);

        visitor.setModularNetworkVisitor(mcnnSTC);

        Log.info("Created Symbol Table","ESTC_INIT");
    }

    private void initSuperSTC(final ResolvingConfiguration resolvingConfig, String customFilesPath, String pythonPath, String backend) {
        Log.info("INIT_SUPER_STC","SSTC_INIT_2");
        this.cnnArchSTC = new CNNArchSymbolTableCreator(resolvingConfig, scopeStack, customFilesPath, pythonPath, backend);
        this.emamSTC = new EmbeddedMontiArcMathSymbolTableCreatorTOP(resolvingConfig, scopeStack);
        this.mathOptSTC = new MathOptSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC = new ModifiedEMADynamicSymbolTableCreator(resolvingConfig, scopeStack);
        this.emadSTC.setInstanceSymbolCreator(new ModifiedEMAComponentInstanceSymbolCreator()); //Use an instance symbo, creator that adds math statement to instances
        this.emaBehaviorSTC = new EmbeddedMontiArcBehaviorSymbolTableCreator(resolvingConfig, scopeStack);
        this.mcnnSTC = new ModularCNNSymbolTableCreator(resolvingConfig, scopeStack, archNodes);

        visitor.setEMADLVisitor(this);
        visitor.setCNNArchVisitor(cnnArchSTC);

        visitor.setEmbeddedMontiArcMathVisitor(emamSTC);
        visitor.setEmbeddedMontiArcVisitor(emadSTC);
        visitor.setEmbeddedMontiArcDynamicVisitor(emadSTC);
        visitor.setEmbeddedMontiArcBehaviorVisitor(emaBehaviorSTC);
        visitor.setMathVisitor(mathOptSTC);
        visitor.setExpressionsBasisVisitor(mathOptSTC);
        visitor.setCommonExpressionsVisitor(mathOptSTC);
        visitor.setAssignmentExpressionsVisitor(mathOptSTC);
        visitor.setMatrixExpressionsVisitor(mathOptSTC);
        visitor.setMatrixVisitor(mathOptSTC);
        visitor.setTypes2Visitor(mathOptSTC);
        visitor.setMathOptVisitor(mathOptSTC);

        visitor.setCommon2Visitor(emamSTC);

        visitor.setModularNetworkVisitor(mcnnSTC);

        Log.info("Created Symbol Table","ESTC_INIT");

    }

/**
     * Creates the symbol table starting from the <code>rootNode</code> and
     * returns the first scope that was created.
     *
     * @param rootNode the root node
     * @return the first scope that was created
     */


    public Scope createFromAST(ASTEMACompilationUnit rootNode) {
        Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EMADLSymbolTableCreator symbol table: top ast node is null");
        rootNode.accept(visitor);

        Log.info("Size of ArchNodes:" + this.archNodes.size(),"ROOT_NODE_ARCH_NODE");

        Log.info(rootNode.toString(),"ROOT_NODE");
        return getFirstCreatedScope();
    }

    @Override
    public MutableScope getFirstCreatedScope() {
        return emadSTC.getFirstCreatedScope();
    }

    private EMADLVisitor realThis = this;

    public EMADLVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EMADLVisitor realThis) {
        if (this.realThis != realThis) {
            this.realThis = realThis;
            visitor.setRealThis(realThis);
        }
    }

    public void endVisit(ASTBehaviorEmbedding ast) {
        if(ast.isPresentArchitecture()){
            //processed in handle/visit/endVisit of ASTArchitecture and ASTArchBody
        }else if(ast.getStatementList().size() > 0) {
            addToScopeAndLinkWithNode(new EMADLMathStatementsSymbol("MathStatements", ast.getStatementList()), ast);
        }
    }

    /*public void visit(ASTBehaviorEmbedding ast){
        Log.info("TEST BE","TEST_VISITOR");
        if (ast.isPresentArchitecture()){
           ASTArchitecture arch =  ast.getArchitecture();
           Collection<de.monticore.ast.ASTNode> children = arch.get_Children();
           for (de.monticore.ast.ASTNode n: children){
               Optional<? extends Symbol> symbolOpt =  n.getSymbolOpt();
               if (!symbolOpt.isPresent()) {
                   Log.info("NO SYMBOL PRESENT","SYMBOL_CHECK");
                   continue;
               }

               Symbol symbol = symbolOpt.get();

               symbol.getName();
               Log.info(symbol.getName(),"SYMBOL_CHECK");
               Log.info(symbol.getFullName(),"SYMBOL_CHECK");
               Log.info(symbol.getPackageName(),"SYMBOL_CHECK");
               Log.info(symbol.getKind().toString(),"SYMBOL_CHECK");

               if (symbol.getAstNode().isPresent()){
                   ASTNode astNode = symbol.getAstNode().get();
                   //astNode.get
               }

           }
        }

    }
*/

   /* public void visit(ASTNode node) {

        Log.info("TEST N","TEST_VISITOR_ASTNode");
        Log.info(node.toString(),"TEST_VISITOR_ASTNode");
        //node.
        if (node.isPresentSpannedScope()) Log.info(node.getSpannedScope().toString(),"TEST_VISITOR_ASTNode");
        if (node.isPresentEnclosingScope()) Log.info(node.getEnclosingScope().toString(),"TEST_VISITOR_ASTNode");

        *//*Optional<? extends Symbol> symbolOpt = node.getSymbolOpt();



        if (!symbolOpt.isPresent()) {
            Log.info("NO SYMBOL PRESENT","SYMBOL_CHECK_ASTNode");
            return;
        }


        Symbol symbol = symbolOpt.get();

        Log.info(symbol.getName(),"SYMBOL_CHECK_ASTNode");
        Log.info(symbol.getFullName(),"SYMBOL_CHECK_ASTNode");
        Log.info(symbol.getPackageName(),"SYMBOL_CHECK_ASTNode");
        Log.info(symbol.getKind().toString(),"SYMBOL_CHECK_ASTNode");
        *//*



    }*/

/*    public void handle(ASTNode node){
        Log.info("TEST N","TEST_HANDLER_ASTNode");
        Log.info(node.toString(),"TEST_HANDLER_ASTNode");
    }

    public void visit(ASTArchitecture node){
        Log.info("TEST N","VISITOR_Rand");
    }*/

}
