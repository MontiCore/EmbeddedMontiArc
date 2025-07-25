/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

public class SerializationException extends Exception {
    private static final long serialVersionUID = 1238068460578236686L;
    
    private final Exception source;

    public SerializationException(Exception source){
        this.source = source;
    }

    @Override
    public String getMessage(){
        return "SerializationException caused by "+source.getClass().getSimpleName()+": "+source.getMessage();
    }
}
