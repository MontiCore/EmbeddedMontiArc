/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbolReference;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbolHelper;
import de.monticore.lang.embeddedmontiarcdynamic.helper.SymbolPrinter;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;



public class EventReferenceExpressionSymbol extends EventExpressionSymbol {

    ComponentEventSymbolReference eventSymbolReference;

    protected List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
    protected List<PortValueSymbol> arguments = new ArrayList<>();


    public EventReferenceExpressionSymbol(String name, ComponentEventSymbolReference eventSymbolReference){
        super(name);
        this.eventSymbolReference = eventSymbolReference;
    }


    public void addActualTypeArgument(ActualTypeArgument ata ){
        actualTypeArguments.add(ata);
    }

    public void addArgument(PortValueSymbol pvs){
        arguments.add(pvs);
    }

    @Override
    public String toString() {

        return super.toString();

    }

    @Override
    public String getTextualRepresentation() {
        StringBuilder sb = new StringBuilder();

        sb.append(getFullName());

        if(actualTypeArguments.size() > 0){
            sb.append(de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter.printTypeParameters(actualTypeArguments));
        }

        if(arguments.size() > 0){
            sb.append("(");
            for (int i = 0; i < arguments.size(); ++i){
                sb.append(arguments.get(i).getTextualRepresentation());
                if(i < arguments.size()-1){
                    sb.append(", ");
                }
            }
            sb.append(")");
        }

        return sb.toString();

//        return super.getTextualRepresentation();
    }

    public EventExpressionSymbol getCondition(){
        return this.eventSymbolReference.getCondition();
    }

    @Override
    public EventExpressionSymbol expand() {

        Map<MCTypeSymbol, ActualTypeArgument> typeArgumentsMap = new LinkedHashMap();
        for(int i = 0; i < this.eventSymbolReference.getFormalTypeParameters().size(); ++i) {
            typeArgumentsMap.put(this.eventSymbolReference.getFormalTypeParameters().get(i), this.actualTypeArguments.get(i));
        }


        Map<EMAVariable, PortValueSymbol> typeParameters = new LinkedHashMap<>();
        for(int i = 0; i < this.eventSymbolReference.getParameters().size(); ++i){
            typeParameters.put(this.eventSymbolReference.getParameters().get(i), this.arguments.get(i));
        }

        EventExpressionSymbol cond = this.eventSymbolReference.getCondition().expand();

//        System.out.println(cond.getTextualRepresentation());

//        this.expandInner(cond, typeArgumentsMap, typeParameters);
        EventExpressionSymbolHelper.getINSTANCE().expandWithArgsAndParams(cond, typeArgumentsMap, typeParameters);

//        System.out.println(cond.getTextualRepresentation());

        return new EventBracketExpressionSymbol(cond);//super.expand();
    }



}
