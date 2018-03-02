package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.ranges._ast.ASTNaturalNumbersOneRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTImportStatement;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.prettyprint.IndentPrinter;

public class EmamToEmaPrettyPrinter implements EmbeddedMontiArcMathVisitor {

    private IndentPrinter ip;
    private String portString = "";

    public static String prettyPrint(ASTEMAMCompilationUnit ast) {
        EmamToEmaPrettyPrinter pP = new EmamToEmaPrettyPrinter();
        pP.ip = new IndentPrinter();
        ast.accept(pP);
        return pP.ip.getContent();
    }

    @Override
    public void visit(ASTEMACompilationUnit node) {
        String packageString = "";
        for (String str : node.getPackage()) {
            packageString += str + ".";
        }
        ip.println("package " + packageString.substring(0, packageString.length() - 1) + ";");

        for (ASTImportStatement importStatement : node.getImportStatements()) {
            String importString = "";
            for (String str : importStatement.getImportList()) {
                importString += str + ".";
            }
            String res = "import " + importString.substring(0, importString.length() - 1);
            if (importStatement.isStar()) res += "*";
            ip.println(res + ";");
        }
    }

    @Override
    public void visit(ASTComponent node) {
        ip.println("component " + node.getName() + " {");
        ip.indent();
    }

    @Override
    public void endVisit(ASTComponent node) {
        ip.unindent();
        ip.println("}");
    }

    @Override
    public void visit(ASTInterface node) {
        ip.println("port");
        ip.indent();
    }

    @Override
    public void endVisit(ASTInterface node) {
        String res = portString.substring(0, portString.lastIndexOf(",\n")) + ";";
        ip.println(res);
        ip.unindent();
    }

    @Override
    public void visit(ASTPort node) {
        String res = "";
        if (node.isIncoming()) res += "in ";
        if (node.isOutgoing()) res += "out ";

        if (node.getType() instanceof ASTElementType &&((ASTElementType) node.getType()).getTElementType().isPresent()){
            res += ((ASTElementType) node.getType()).getTElementType().get() + " ";
        } else if(node.getType() instanceof ASTCommonMatrixType ) {
            res += ((ASTCommonMatrixType) node.getType()).getElementType().getTElementType().get();
            res += (" ^ {" + ((ASTCommonMatrixType) node.getType()).getElementType().getRange().
                    get().getStep().get().getUnitNumber().get()).replace("/",",") + "} ";
        } else if(node.getType() instanceof ASTElementType){
            if (((ASTElementType) node.getType()).isIsBoolean()) res += "B ";
            if (((ASTElementType) node.getType()).isIsComplex()) res += "Z ";
            if (((ASTElementType) node.getType()).isIsNatural()) res += "N ";
            if (((ASTElementType) node.getType()).isIsRational()) res += "Q ";
        } else if (node.getType() instanceof ASTNaturalNumbersOneRange){
            if(((ASTNaturalNumbersOneRange) node.getType()).stepIsPresent()){
                if( ((ASTNaturalNumbersOneRange) node.getType()).getStep().get().getUnitNumber().get().
                        getInfSign().get()){
                    res += "N1 ";
                }
            }
        } else {
            System.out.println("Unknown port type: " + node.getType().getClass());
        }

        res += node.getName().get() + ",\n";

        portString += res;
    }

    @Override
    public void visit(ASTSubComponent node) {
        String res = "instance " + ((ASTSimpleReferenceType) node.getType()).getNames().get(0) + " ";

        boolean first = true;
        for (ASTSubComponentInstance instance : node.getInstances()) {
            if (first) first = false;
            else res += ", ";
            res += instance.getName();
        }
        res += ";";

        ip.println(res);
    }

    @Override
    public void visit(ASTConnector node) {
        String res = "connect ";

        if(node.getSource().get().compNameIsPresent())
            res += node.getSource().get().getCompName().get() + ".";
        res += node.getSource().get().getPortName() + " -> ";

        for(ASTQualifiedNameWithArray target : node.getTargets().getQualifiedNameWithArrays()){
            if(target.getCompName().isPresent())
                res += target.getCompName().get() + ".";
            res += target.getPortName() + ", ";
        }
        res = res.substring(0, res.length() - 2) + ";";

        ip.println(res);
    }

}
