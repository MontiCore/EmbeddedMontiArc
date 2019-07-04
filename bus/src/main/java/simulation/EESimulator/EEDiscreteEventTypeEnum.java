package simulation.EESimulator;

public enum EEDiscreteEventTypeEnum {
    
	BUSMESSAGE("BUSMESSAGE"),
    KEEP_ALIVE_EVENT("KEEP_ALIVE_EVENT");

    private final String name;

    private EEDiscreteEventTypeEnum(String name){
        this.name = name;
    }


    public String toString(){
        return this.name;
    }
}
