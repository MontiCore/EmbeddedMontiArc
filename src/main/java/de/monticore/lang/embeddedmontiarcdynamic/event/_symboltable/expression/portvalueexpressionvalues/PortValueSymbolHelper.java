/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;

import java.util.Map;

public class PortValueSymbolHelper {

    //<editor-fold desc="Single Instance">
    protected static PortValueSymbolHelper INSTANCE = null;
    public static PortValueSymbolHelper getINSTANCE(){
        if(INSTANCE == null){
            PortValueSymbolHelper.init();
        }
        return INSTANCE;
    }
    protected void setINSTANCE(PortValueSymbolHelper builder){
        INSTANCE = builder;
    }
    public static PortValueSymbolHelper init(){
        PortValueSymbolHelper b = new PortValueSymbolHelper();
        b.setINSTANCE(b);
        return b;
    }
    //</editor-fold>


    public PortValueSymbol expand(PortValueSymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        if(pvs instanceof PortValueCompareSymbol){
            return expand_PortValueCompareSymbol((PortValueCompareSymbol)pvs, params);
        }

        if(pvs instanceof PortValueInputSymbol){
            return expand_PortValueInputSymbol((PortValueInputSymbol)pvs, params);
        }

        if(pvs instanceof PortValuePrecisionSymbol){
            return expand_PortValuePrecisionSymbol((PortValuePrecisionSymbol) pvs, params);
        }

        if(pvs instanceof PortValueRangeSymbol){
            return expand_PortValueRangeSymbol((PortValueRangeSymbol) pvs,params);
        }

        if(pvs instanceof PortValuesArraySymbol){
            return expand_PortValuesArraySymbol((PortValuesArraySymbol) pvs, params);
        }

        if(pvs instanceof PortValuesMatrixSymbol){
            return expand_PortValuesMatrixSymbol((PortValuesMatrixSymbol) pvs, params);
        }
        return null;
    }

    protected PortValueCompareSymbol expand_PortValueCompareSymbol(PortValueCompareSymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        return new PortValueCompareSymbol(pvs.operator, (PortValueInputSymbol) expand_PortValueInputSymbol(pvs.getCompareValue(), params));
    }

    protected PortValueSymbol expand_PortValueInputSymbol(PortValueInputSymbol pvs, Map<EMAVariable, PortValueSymbol> params){

        if(pvs.isLogic()){
            return new PortValueInputSymbol(pvs.logicValue);
        }

        if(pvs.isNumber()){
            return new PortValueInputSymbol(pvs.numberValue, pvs.numberIsPosInf, pvs.numberIsNegInf);
        }

        if(pvs.isVariable()){
            PortValueSymbol pvis = null;

            for (Map.Entry<EMAVariable, PortValueSymbol> entry: params.entrySet()) {
                if(pvs.getVariableValue().equals(entry.getKey().getName())){
                    pvis = entry.getValue();
                    break;
                }
            }

            if(pvis == null){
                pvis = new PortValueInputSymbol(pvs.getVariableValue());
            }
            return pvis;
        }
        return null;
    }

    protected PortValuePrecisionSymbol expand_PortValuePrecisionSymbol(PortValuePrecisionSymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        if(pvs.hasPrecision()){
            return new PortValuePrecisionSymbol((PortValueInputSymbol) expand_PortValueInputSymbol(pvs.getValue(),params),
                                                (PortValueInputSymbol) expand_PortValueInputSymbol(pvs.getPrecision(),params));
        }else{
            return new PortValuePrecisionSymbol((PortValueInputSymbol) expand_PortValueInputSymbol(pvs.getValue(),params));
        }
    }

    protected PortValueRangeSymbol expand_PortValueRangeSymbol(PortValueRangeSymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        return new PortValueRangeSymbol((PortValueInputSymbol)expand_PortValueInputSymbol(pvs.lowerBound, params),
                                        (PortValueInputSymbol)expand_PortValueInputSymbol(pvs.upperBound,params));
    }

    protected PortValuesArraySymbol expand_PortValuesArraySymbol(PortValuesArraySymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        PortValuesArraySymbol pa = new PortValuesArraySymbol();
        for (int i = 0; i < pvs.size(); ++i) {
            pa.add(expand(pvs.getForIndex(i), params));
        }
        return pa;
    }

    protected PortValuesMatrixSymbol expand_PortValuesMatrixSymbol(PortValuesMatrixSymbol pvs, Map<EMAVariable, PortValueSymbol> params){
        PortValuesMatrixSymbol ms = new PortValuesMatrixSymbol();
        for(int i = 0; i < ms.size(); ++i){
            ms.add(expand_PortValuesArraySymbol(ms.getArraySymbolForIndex(i), params));
        }
        return ms;
    }
}
