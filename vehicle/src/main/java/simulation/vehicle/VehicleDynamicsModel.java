package simulation.vehicle;

import org.javafmi.wrapper.Simulation;

/**
 * Class that contains the vehicle dynamics model and methods to safely interact with it
 */
public class VehicleDynamicsModel {

    /** FMU that represents the inputFilter of the car*/
    private Simulation inputFilter;

    /** FMU that represents the chassis of the car*/
    private Simulation chassis;

    /** FMU that represents the suspension of the car*/
    private Simulation suspension;

    /** FMU that represents the tires of the car*/
    private Simulation tires;

    /** Flag whether the VDM has been initialized */
    private boolean isInitialized;

    /** Flag whether the VDM has been terminated */
    //private boolean isTerminated = false;

    /** Flag whether the FDUs need to exchange their values */
    private boolean needsExchanging;

    /**
     * Constructor for an uninitialized VDM
     */
    public VehicleDynamicsModel(){
        inputFilter = new Simulation("lib/InputFilter.fmu");
        chassis = new Simulation("lib/Chassis.fmu");
        suspension = new Simulation("lib/Suspension.fmu");
        tires = new Simulation("lib/Tires.fmu");
        isInitialized = false;
        needsExchanging= false;
    }

    /**
     * Function that initialized the VDM
     * Should only be called by physicalVehicleBuilder
     */
    public void initialize(){
        if(!isInitialized) {
            double stopTime = 10.0;
            inputFilter.init(0, stopTime);
            chassis.init(0, stopTime);
            suspension.init(0, stopTime);
            tires.init(0, stopTime);
            isInitialized = true;
            exchangeValues();
        }else{
            //ToDo error if already initialized
        }
    }

    /**
     * Function that executes one computation step
     * @param stepSize Size of the computation step
     */
    public void doStep(double stepSize){
        if(isInitialized) {
            //if (!isTerminated) {
            //    inputFilter.doStep(stepSize);
            //} else {
            //    //ToDo error if already terminated
            //}
            if(needsExchanging){
                exchangeValues();
            }
            inputFilter.doStep(stepSize);
            chassis.doStep(stepSize);
            suspension.doStep(stepSize);
            tires.doStep(stepSize);
            exchangeValues();
        }else{
            //ToDo error if not initialized
        }
    }

    /**
     * Function that sets a value into the corresponding FMU
     * @param name Name of the value to be set
     * @param value New value to be set
     */
    public void setValue(String name, double value){
        if(isInitialized) {
            switch (name) {
                case "bank":
                case "slope":
                    inputFilter.write(name).with(value);
                    break;
                case "tau_D_1":
                case "tau_D_2":
                case "tau_D_3":
                case "tau_D_4":
                case "tau_B_1":
                case "tau_B_2":
                case "tau_B_3":
                case "tau_B_4":
                case "F_ext_x":
                case "F_ext_y":
                    chassis.write(name).with(value);
                    break;
                case "delta_1":
                case "delta_2":
                    chassis.write(name).with(value);
                    tires.write(name).with(value);
                    break;
                default:
                    System.out.println(name + " wants to be written!");
                    break;
            }
            needsExchanging = true;
        }else{
            //todo error if not initialized
        }
    }

    /**
     * Function that gets a value from the corresponding FMU
     * @param name Name of the value to be returned
     * @return Value with the given name
     */
    public double getValue(String name){
        double value = 0.0;
        if(isInitialized) {
            if (needsExchanging) {
                exchangeValues();
            }
            switch (name) {
                case "r_nom":
                case "m":
                case "omega_wheel_1":
                case "omega_wheel_2":
                case "omega_wheel_3":
                case "omega_wheel_4":
                case "z":
                case "omega_z":
                case "v_x":
                case "v_y":
                case "L_1":
                case "L_2":
                case "TW_f":
                case "TW_r":
                    value = chassis.read(name).asDouble();
                    break;
                default:
                    System.out.println(name + " wants to be read!");
                    break;
            }
        }else{
            //todo error if not initialized
        }
        return value;
    }

    /**
     * Function that exchanges the values under the FMUs
     */
    private void exchangeValues(){
        chassis.write("slope_d").with(inputFilter.read("slope_d").asDouble());
        chassis.write("bank_d").with(inputFilter.read("bank_d").asDouble());
        suspension.write("slope_d").with(inputFilter.read("slope_d").asDouble());
        suspension.write("bank_d").with(inputFilter.read("bank_d").asDouble());
        suspension.write("a_y").with(chassis.read("a_y").asDouble());
        suspension.write("pitch_angle").with(chassis.read("pitch_angle").asDouble());
        suspension.write("omega_y").with(chassis.read("omega_y").asDouble());
        tires.write("v_s_1").with(chassis.read("v_s_1").asDouble());
        tires.write("v_s_2").with(chassis.read("v_s_2").asDouble());
        tires.write("v_s_3").with(chassis.read("v_s_3").asDouble());
        tires.write("v_s_4").with(chassis.read("v_s_4").asDouble());
        tires.write("v_x_1").with(chassis.read("v_x_1").asDouble());
        tires.write("v_x_2").with(chassis.read("v_x_2").asDouble());
        tires.write("v_x_3").with(chassis.read("v_x_3").asDouble());
        tires.write("v_x_4").with(chassis.read("v_x_4").asDouble());
        tires.write("v_y_1").with(chassis.read("v_y_1").asDouble());
        tires.write("v_y_2").with(chassis.read("v_y_2").asDouble());
        tires.write("v_y_3").with(chassis.read("v_y_3").asDouble());
        tires.write("v_y_4").with(chassis.read("v_y_4").asDouble());
        chassis.write("d_roll").with(suspension.read("d_roll").asDouble());
        chassis.write("d_pitch").with(suspension.read("d_pitch").asDouble());
        chassis.write("K_roll_f").with(suspension.read("K_roll_f").asDouble());
        chassis.write("K_roll_r").with(suspension.read("K_roll_r").asDouble());
        chassis.write("D_roll_f").with(suspension.read("D_roll_f").asDouble());
        chassis.write("D_roll_r").with(suspension.read("D_roll_r").asDouble());
        chassis.write("D_pitch").with(suspension.read("D_pitch").asDouble());
        chassis.write("K_pitch").with(suspension.read("K_pitch").asDouble());
        tires.write("F_z_1").with(suspension.read("F_z_1").asDouble());
        tires.write("F_z_2").with(suspension.read("F_z_2").asDouble());
        tires.write("F_z_3").with(suspension.read("F_z_3").asDouble());
        tires.write("F_z_4").with(suspension.read("F_z_4").asDouble());
        chassis.write("F_x_1").with(tires.read("F_x_1").asDouble());
        chassis.write("F_x_2").with(tires.read("F_x_2").asDouble());
        chassis.write("F_x_3").with(tires.read("F_x_3").asDouble());
        chassis.write("F_x_4").with(tires.read("F_x_4").asDouble());
        chassis.write("F_y_1").with(tires.read("F_y_1").asDouble());
        chassis.write("F_y_2").with(tires.read("F_y_2").asDouble());
        chassis.write("F_y_3").with(tires.read("F_y_3").asDouble());
        chassis.write("F_y_4").with(tires.read("F_y_4").asDouble());
        needsExchanging = false;
    }
}