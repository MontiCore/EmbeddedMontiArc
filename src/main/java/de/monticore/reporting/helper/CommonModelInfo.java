/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

public abstract class CommonModelInfo {

    private File modelFile = null;
    private String modelName = "";
    private String qualifiedName = "";
    private String modelPath = "";
    private String fileType = "";
    private String gitLabLink = "";
    private int parsed = 0;
    private int resolved = 0;

    private ASTEMACompilationUnit unresolvedAST = null;
    private EMAComponentSymbol resolvedAST = null;
    private ASTEmbeddedMontiArcNode resolvedASTNode = null;

    private List<String> errorMessage = new LinkedList<>();

    public CommonModelInfo(File modelFile) {
        this.modelFile = modelFile;
    }

    public File getModelFile() {
        return modelFile;
    }

    public void setModelFile(File modelFile) {
        this.modelFile = modelFile;
    }

    public String getModelFileAsString() {
        return modelFile.getAbsolutePath();
    }

    public String getModelName() {
        return modelName;
    }

    public void setModelName(String modelName) {
        this.modelName = modelName;
    }

    public String getModelPath() {
        return modelPath;
    }

    public void setModelPath(String modelPath) {
        this.modelPath = modelPath;
    }

    public String getFileType() {
        return fileType;
    }

    public void setFileType(String fileType) {
        this.fileType = fileType;
    }

    public int getParsed() {
        return parsed;
    }

    public void setParsed(int parsed) {
        this.parsed = parsed;
    }

    public int getResolved() {
        return resolved;
    }

    public void setResolved(int resolved) {
        this.resolved = resolved;
    }

    public ASTEMACompilationUnit getUnresolvedAST() {
        return unresolvedAST;
    }

    public void setUnresolvedAST(ASTEMACompilationUnit unresolvedAST) {
        this.unresolvedAST = unresolvedAST;
    }

    public EMAComponentSymbol getResolvedAST() {
        return resolvedAST;
    }

    public void setResolvedAST(EMAComponentSymbol resolvedAST) {
        this.resolvedAST = resolvedAST;
    }

    public List<String> getErrorMessages() {
        return errorMessage;
    }

    public abstract String getErrorMessage();

    public void addErrorMessage(String errorMessage) {
        this.errorMessage.add(errorMessage);
    }

    public String getQualifiedName() {
        return qualifiedName;
    }

    public void setQualifiedName(String qualifiedName) {
        this.qualifiedName = qualifiedName;
    }

    public ASTEmbeddedMontiArcNode getResolvedASTNode() {
        return resolvedASTNode;
    }

    public void setResolvedASTNode(ASTEmbeddedMontiArcNode resolvedASTNode) {
        this.resolvedASTNode = resolvedASTNode;
    }

    public String getGitLabLink() {
        return gitLabLink;
    }

    public void setGitLabLink(String gitLabLink) {
        this.gitLabLink = gitLabLink;
    }
}
