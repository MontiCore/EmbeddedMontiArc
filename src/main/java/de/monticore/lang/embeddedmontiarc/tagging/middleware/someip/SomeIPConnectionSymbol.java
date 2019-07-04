package de.monticore.lang.embeddedmontiarc.tagging.middleware.someip;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.tagging._symboltable.TagKind;

import java.util.Optional;

public class SomeIPConnectionSymbol extends MiddlewareSymbol {

    public static final SomeIPConnectionKind KIND = SomeIPConnectionKind.INSTANCE;

    public SomeIPConnectionSymbol() {
        super(KIND, Optional.empty(), Optional.empty(), Optional.empty());
    }

    public SomeIPConnectionSymbol(String id) {
        this(KIND, id);
    }

    public SomeIPConnectionSymbol(SomeIPConnectionKind kind, String id) {
        super(kind, Optional.ofNullable(id));
    }

    public SomeIPConnectionSymbol(String serviceID, String instanceID){
        this(KIND, serviceID, instanceID);
    }

    public SomeIPConnectionSymbol(String serviceID, String instanceID, String eventgroupID) {
        this(KIND, serviceID, instanceID, eventgroupID);
    }

    protected SomeIPConnectionSymbol(SomeIPConnectionKind kind, String serviceID, String instanceID, String eventgroupID) {
        super(kind, Optional.ofNullable(serviceID), Optional.ofNullable(instanceID), Optional.ofNullable(eventgroupID));
    }

    @Override
    public String toString() {
        return String.format("SomeIPConnection = %s, %s, %s",
                getserviceID(), getinstanceID(), geteventgroupID());
    }

    public Optional<String> getserviceID() {
        return getValue(0);
    }

    public Optional<String> getinstanceID() {
        return getValue(1);
    }

    public Optional<String> geteventgroupID() {
        return getValue(2);
    }

    public void setserviceID(String serviceID) {
        this.values.set(0, Optional.ofNullable(serviceID));
    }

    public void setinstanceID(String instanceID) {
        this.values.set(1, Optional.ofNullable(instanceID));
    }

    public void seteventgroupID(String eventgroupID) {
        this.values.set(2, Optional.ofNullable(eventgroupID));
    }

    public static class SomeIPConnectionKind extends TagKind {
        public static final SomeIPConnectionKind INSTANCE = new SomeIPConnectionKind();

        protected SomeIPConnectionKind() {
        }
    }
}
