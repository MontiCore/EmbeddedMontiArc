/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTComponentEvent;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import org.antlr.v4.runtime.ParserRuleContext;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;

public class ComponentEventSymbol extends ComponentEventSymbolTOP {



    protected EventExpressionSymbol condition;
    protected List<EMAVariable> parameters = new ArrayList<>();

    public ComponentEventSymbol(String name){
        super(name);
    }

    public void /*List<NamedStreamUnitsSymbol>*/ getNamedStreams() {
        //return new ArrayList<>(getSpannedScope().<NamedStreamUnitsSymbol>resolveLocally(NamedStreamUnitsSymbol.KIND));
        //getSpannedScope().<ParserRuleContext>resolveLocally()
    }


    public String getComponentName(){
        if(getAstNode().isPresent()){
            ASTComponentEvent astce = (ASTComponentEvent)getAstNode().get();
            return astce.getName();
        }
        return "";
    }

    /*public ASTExpression getExpression(){
        if(getAstNode().isPresent()){
            ASTComponentEvent astce = (ASTComponentEvent)getAstNode().get();
            return astce.getExpression();
        }
        return null;
    }*/

    public void setCondition(EventExpressionSymbol expressionSymbol){
        this.condition = expressionSymbol;
    }

    public EventExpressionSymbol getCondition(){
        return this.condition;
    }

    public String toNiceString(){
        StringBuilder sb = new StringBuilder();
        sb.append("package TODO ADD PACKAGE");
        sb.append(System.lineSeparator());
        sb.append(System.lineSeparator());

        sb.append("event ");
        sb.append(this.getName());

        sb.append("/* TODO: types, parameter, for component */");

        sb.append("{");
        sb.append(System.lineSeparator());
        sb.append("\t");
        sb.append(this.getCondition().getTextualRepresentation());
        sb.append(System.lineSeparator());
        sb.append("}");
        return sb.toString();
    }


    public void addFormalTypeParameter(MCTypeSymbol formalTypeParameter) {
//        if (referencedComponent.isPresent()) {
//            referencedComponent.get().addFormalTypeParameter(formalTypeParameter);
//        } else {
            checkArgument(formalTypeParameter.isFormalTypeParameter());
            getMutableSpannedScope().add(formalTypeParameter);
//        }
    }

    public List<MCTypeSymbol> getFormalTypeParameters() {
        final Collection<MCTypeSymbol> resolvedTypes = this.getSpannedScope().resolveLocally(MCTypeSymbol.KIND);
        return resolvedTypes.stream().filter(MCTypeSymbol::isFormalTypeParameter)
                .collect(Collectors.toList());
    }

    public boolean hasFormalTypeParameters() {
        return !getFormalTypeParameters().isEmpty();
    }


    public void addParameter(ASTParameter astParameter) {
            EMAVariable param = new EMAVariable();
            param.setName(astParameter.getNameWithArray().getName());
            param.setType(astParameter.getType());
            parameters.add(param);
    }

    public List<EMAVariable> getParameters() {
        return parameters;
    }


}
