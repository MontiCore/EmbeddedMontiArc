/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.symboltable.Symbol;

import java.util.Optional;

public class EventPortExpression extends EventExpressionSymbol {

    //<editor-fold desc="Porperties">

    protected String portName;
    protected Optional<String> componentName;
    protected boolean hasPortArray = false;
    protected boolean hasPortArraySingleValue = false;
    protected int portArraySingleValue = 0;
    protected boolean hasPortArrayLowerAndUpperBounds = false;
    protected int portArrayLowerBound = 0;
    protected int portArrayUpperBound = 0;

    //</editor-fold>

    public EventPortExpression(){
        super();
        this.portName = "";
        this.componentName = Optional.empty();
    }

    public void setComponentName(String name) {
        this.componentName= Optional.of(name);
    }

    public String getComponentName() {
        return componentName.get();
    }

    public Optional<String> getComponentNameOpt() {
        return componentName;
    }

    public boolean hasComponentName() {
        return componentName.isPresent();
    }

    public void setPortName(String portName) {
        this.portName = portName;
    }

    public String getPortName() {
        return this.portName;
    }

    public boolean hasPortArray() {
        return this.hasPortArray;
    }

    public boolean hasPortArrayLowerAndUpperBound() {
        return this.hasPortArrayLowerAndUpperBounds;
    }

    public void setHasPortArray(boolean hasPortArray) {
        this.hasPortArray = hasPortArray;
    }

    public void setPortArrayLowerAndUpperBounds(int lower, int upper) {
        this.setHasPortArray(true);
        this.hasPortArrayLowerAndUpperBounds = true;
        this.portArrayLowerBound = lower;
        this.portArrayUpperBound = upper;
    }

    public int getPortArrayLowerBound() {
        return portArrayLowerBound;
    }

    public int getPortArrayUpperBound() {
        return portArrayUpperBound;
    }

    public boolean hasPortArraySingleValue(){
        return this.hasPortArraySingleValue;
    }

    public void setPortArraySingleValue(int value){
        this.setHasPortArray(true);
        this.hasPortArrayLowerAndUpperBounds = false;
        this.hasPortArraySingleValue = true;
        this.portArraySingleValue = value;
    }

    public int getPortArraySingleValue(){
        return this.portArraySingleValue;
    }

    @Override
    public String getTextualRepresentation() {
        StringBuilder sb = new StringBuilder();
        if(this.componentName.isPresent()){
            sb.append(this.componentName.get());
            sb.append(".");
        }
        sb.append(portName);
        if(this.hasPortArray){
            sb.append("[");
            if(this.hasPortArrayLowerAndUpperBounds) {
                sb.append(this.portArrayLowerBound);
                sb.append(":");
                sb.append(this.portArrayUpperBound);
            }else if(this.hasPortArraySingleValue){
                sb.append(this.portArraySingleValue);
            }else{
                sb.append(":");
            }
            sb.append("]");
        }
        return sb.toString();
    }

    @Override
    public EventExpressionSymbol expand() {
        EventPortExpression epe = new EventPortExpression();

        this.expand_setProperties(epe);

        return epe;
    }

    protected void expand_setProperties(EventPortExpression epe){
        epe.portName = this.portName;
        epe.componentName = this.componentName;
        epe.hasPortArray = this.hasPortArray;
        epe.hasPortArraySingleValue = this.hasPortArraySingleValue;
        epe.portArraySingleValue = this.portArraySingleValue;
        epe.hasPortArrayLowerAndUpperBounds = this.hasPortArrayLowerAndUpperBounds;
        epe.portArrayLowerBound = this.portArrayLowerBound;
        epe.portArrayUpperBound = this.portArrayUpperBound;
    }

    @Override
    public String getName() {

        String name = this.getPortName();
        if(this.hasComponentName()){
            name = this.componentName.get()+"."+name;
        }

        if(this.hasPortArray()){
            name += "["+this.getPortArraySingleValue()+"]";
        }

        return name;
    }
}
