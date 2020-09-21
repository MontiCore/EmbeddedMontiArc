/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import de.monticore.commonexpressions._ast.ASTNotEqualsExpression;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.*;
import de.se_rwth.commons.logging.Log;

public class PortValueSymbolBUILDER {

    //<editor-fold desc="Builder for PortValueSymbol">

    public static PortValueSymbol build(ASTPortValue node){

        if(node.getSymbolOpt().isPresent()){
            return (PortValueSymbol)node.getSymbolOpt().get();
        }

        PortValueSymbol pvs = null;
        if(node.isPresentPortSingleValue()){
            pvs = buildSingleValue(node.getPortSingleValue());
        }else if(node.isPresentPortArrayValue()){
            pvs = buildArrayValue(node.getPortArrayValue());
        }

        if(!node.getSymbolOpt().isPresent() && pvs != null) {
            node.setSymbol(pvs);
        }
        return pvs;
    }

    public static PortValueSymbol buildSingleValue(ASTPortSingleValue node){
        if(node.getSymbolOpt().isPresent()){
            return (PortValueSymbol)node.getSymbolOpt().get();
        }
        PortValueSymbol pvs = null;
        if(node.isPresentNumberWithPrecision()) {
            pvs =  handleASTNumberWithPrecision(node.getNumberWithPrecision());

        }else if(node.isPresentNumberRange()) {
            pvs = handleASTNumberRange(node.getNumberRange());
//        }else if(node.isPresentCompareToValue()) {
//            pvs = handleASTCompareToValue(node.getCompareToValue());
//        }
        }else if(node.isPresentCTV()){
            pvs = handleASTCompareToValue(node.getCTV().getCompareToValue());
        }

        node.setSymbol(pvs);

        return pvs;
    }

    public static PortValueSymbol buildArrayValue(ASTPortArrayValue node){
        if(node.getSymbolOpt().isPresent()){
            return (PortValueSymbol)node.getSymbolOpt().get();
        }
        PortValueSymbol pvs = null;

        if(node.isPresentPortArrayValueContent()){
            pvs =  handleASTPortArrayValueContent(node.getPortArrayValueContent());
        }else if(node.isPresentPortArrayValueMatrixContent()){
            pvs =  handleASTPortArrayValueMatrixContent(node.getPortArrayValueMatrixContent());
        }

        node.setSymbol(pvs);

        return pvs;
    }

    //</editor-fold>

    //<editor-fold desc="Handler for different Value Types">

    protected static PortValueInputSymbol handleASTValueInput(ASTValueInput node){
        PortValueInputSymbol pvis = null;
        if(node.isPresentTrueExpression()){
            return new PortValueInputSymbol(true);
        }else if(node.isPresentFalseExpression()){
            return new PortValueInputSymbol(false);
        }else if(node.isPresentNumberWithUnit()){
            if(node.getNumberWithUnitOpt().get().isMinusInfinite()){
                pvis = new PortValueInputSymbol(0.0, false, true);
            }else if(node.getNumberWithUnitOpt().get().isPlusInfinite()){
                pvis = new PortValueInputSymbol(0.0, true, false);
            }else{
                pvis = new PortValueInputSymbol(node.getNumberWithUnit().getNumber().get());
            }
            pvis.setEmaValueWithASTNumberWithUnit(node.getNumberWithUnit());
            return pvis;
        }else if(node.isPresentNameWithArray()){
            return new PortValueInputSymbol(node.getNameWithArray().getName());
        }
        return null;
    }

    protected static PortValueSymbol handleASTNumberWithPrecision(ASTNumberWithPrecision node){
        PortValueInputSymbol pvis = handleASTValueInput(node.getValueInput());


        if(node.isPresentPrecision()){
            PortValuePrecisionSymbol pvps = new PortValuePrecisionSymbol(pvis);;
            pvis = handleASTValueInput(node.getPrecision());
            pvps.setPrecision(pvis);
            return pvps;
        }else{
            return  pvis;
        }
//        return pvps;
    }

    protected static PortValueRangeSymbol handleASTNumberRange(ASTNumberRange node){

        return new PortValueRangeSymbol(
                handleASTValueInput(node.getLowerBound()),
                handleASTValueInput(node.getUpperBound()));

    }

    protected static PortValueCompareSymbol handleASTCompareToValue(ASTCompareToValue node){

        ASTCompareToValue c = node;
//        if(node.isPresentCompareToValueGreater()){
//            c = node.getCompareToValueGreater();
//        }else if(node.isPresentCompareToValueGreaterEquals()){
//            c = node.getCompareToValueGreaterEquals();
//        }else if(node.isPresentCompareToValueLower()){
//            c = node.getCompareToValueLower();
//        }else if(node.isPresentCompareToValueLowerEquals()){
//            c = node.getCompareToValueLowerEquals();
//        }else if(node.isPresentCompareToValueNotEquals()){
//            c = node.getCompareToValueNotEquals();
//        }


        if(c == null){
            Log.error("Unhandled compare ?!", node.get_SourcePositionStart());
        }

        PortValueInputSymbol pvis = handleASTValueInput(c.getCompare());
        return new PortValueCompareSymbol(
                c.getOperator(),
                pvis);
    }

    protected static PortValuesArraySymbol handleASTPortArrayValueContent(ASTPortArrayValueContent node){
        PortValuesArraySymbol pvas = new PortValuesArraySymbol();
        for(ASTPortSingleValue psv : node.getPortSingleValueList()){
            pvas.add(buildSingleValue(psv));
        }
        return pvas;
    }

    protected static PortValuesMatrixSymbol handleASTPortArrayValueMatrixContent(ASTPortArrayValueMatrixContent node){
        PortValuesMatrixSymbol pvms = new PortValuesMatrixSymbol();

        for (ASTPortArrayValueContent pavc : node.getPortArrayValueContentList()) {
            pvms.add(handleASTPortArrayValueContent(pavc));
        }

        return pvms;
    }

    //</editor-fold>

}
