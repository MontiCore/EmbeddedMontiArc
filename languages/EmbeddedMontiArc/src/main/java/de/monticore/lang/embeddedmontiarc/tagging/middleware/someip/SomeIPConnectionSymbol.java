/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.someip;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.tagging._symboltable.TagKind;

import java.util.Optional;

public class SomeIPConnectionSymbol extends MiddlewareSymbol {

    public static final SomeIPConnectionKind KIND = SomeIPConnectionKind.INSTANCE;

    public SomeIPConnectionSymbol() {
        super(KIND, Optional.empty(), Optional.empty(), Optional.empty());
    }

    public SomeIPConnectionSymbol(int serviceID, int instanceID){
        this(KIND, serviceID, instanceID);
    }

    public SomeIPConnectionSymbol(int serviceID, int instanceID, int eventgroupID) {
        this(KIND, serviceID, instanceID, eventgroupID);
    }

    protected SomeIPConnectionSymbol(SomeIPConnectionKind kind, int serviceID, int instanceID, int eventgroupID) {
        super(kind, Optional.ofNullable(serviceID), Optional.ofNullable(instanceID), Optional.ofNullable(eventgroupID));
    }
    
    protected SomeIPConnectionSymbol(SomeIPConnectionKind kind,int serviceID, int instanceID) {
        super(kind, Optional.ofNullable(serviceID), Optional.ofNullable(instanceID), Optional.empty());
    }

    @Override
    public String toString() {
        return String.format("SomeIPConnection = %d, %d, %d",
                getserviceID(), getinstanceID(), geteventgroupID());
    }

    public Optional<Integer> getserviceID() {
        return getValue(0);
    }

    public Optional<Integer> getinstanceID() {
        return getValue(1);
    }

    public Optional<Integer> geteventgroupID() {
        return getValue(2);
    }

    public void setserviceID(int serviceID) {
        this.values.set(0, Optional.ofNullable(serviceID));
    }

    public void setinstanceID(int instanceID) {
        this.values.set(1, Optional.ofNullable(instanceID));
    }

    public void seteventgroupID(int eventgroupID) {
        this.values.set(2, Optional.ofNullable(eventgroupID));
    }

    public static class SomeIPConnectionKind extends TagKind {
        public static final SomeIPConnectionKind INSTANCE = new SomeIPConnectionKind();

        protected SomeIPConnectionKind() {
        }
    }
}
