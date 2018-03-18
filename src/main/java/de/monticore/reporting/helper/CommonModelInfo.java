package de.monticore.reporting.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

public abstract class CommonModelInfo {

    private File modelFile = null;
    private String modelName = "";
    private String qualifiedName = "";
    private String modelPath = "";
    private String fileType = "";
    private int parsed = 0;
    private int resolved = 0;

    private ASTEMAMCompilationUnit unresolvedAST = null;
    private ASTEmbeddedMontiArcNode resolvedAST = null;

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

    public ASTEMAMCompilationUnit getUnresolvedAST() {
        return unresolvedAST;
    }

    public void setUnresolvedAST(ASTEMAMCompilationUnit unresolvedAST) {
        this.unresolvedAST = unresolvedAST;
    }

    public ASTEmbeddedMontiArcNode getResolvedAST() {
        return resolvedAST;
    }

    public void setResolvedAST(ASTEmbeddedMontiArcNode resolvedAST) {
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
}
