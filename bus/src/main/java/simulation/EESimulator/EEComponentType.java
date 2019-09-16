/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

public enum EEComponentType{

    BUS("BUS"),
    SENSOR("SENSOR"),
    BRIDGE("BRIDGE"),
    ACTUATOR("ACTUATOR"),
    AUTOPILOT("AUTOPILOT"),
	TEST_COMPONENT("TEST_COMPONENT"),
    NAVIGATION("NAVIGATION");

    private final String name;

    private EEComponentType(String name){
        this.name = name;
    }


    public String toString(){
        return this.name;
    }

}
