/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

public enum EEDiscreteEventTypeEnum {
    
	BUSMESSAGE("BUSMESSAGE"),
    KEEP_ALIVE_EVENT("KEEP_ALIVE_EVENT"),
    CONTROLLER_EXECUTE_EVENT("CONTROLLER_EXECUTE_EVENT");

    private final String name;

    private EEDiscreteEventTypeEnum(String name){
        this.name = name;
    }


    public String toString(){
        return this.name;
    }
}
