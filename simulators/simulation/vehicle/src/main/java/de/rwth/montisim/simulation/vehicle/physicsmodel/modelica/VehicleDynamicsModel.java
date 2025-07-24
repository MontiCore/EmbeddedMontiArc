/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.modelica;

// TODO

import de.rwth.montisim.commons.utils.LibraryService;
import org.javafmi.wrapper.Simulation;

/**
 * Class that contains the vehicle dynamics model and methods to safely interact with it
 */
public class VehicleDynamicsModel {

    /**
     * FMUs for the car.
     */
    private Simulation inputFilter;
    private Simulation chassis;
    private Simulation suspension;
    private Simulation tires;
    private Simulation brakeSystem;
    private Simulation driveline;
    private Simulation steering;

    private boolean isInitialised;

    private boolean needsExchanging;

    /**
     * Constructor for an uninitialised VDM
     */
    public VehicleDynamicsModel() {
        inputFilter = loadSimulation("lib/InputFilter.fmu");
        chassis = loadSimulation("lib/Chassis.fmu");
        suspension = loadSimulation("lib/Suspension.fmu");
        tires = loadSimulation("lib/Tires.fmu");
        brakeSystem = loadSimulation("lib/BrakeSystem.fmu");
        driveline = loadSimulation("lib/Driveline.fmu");
        steering = loadSimulation("lib/Steering.fmu");
        isInitialised = false;
        needsExchanging = false;
    }

    private Simulation loadSimulation(String name) {
        try {
            LibraryService.prepareLibrary(name, false);
        } catch (LibraryService.LibraryException e) {
            e.printStackTrace();
        }
        return new Simulation(LibraryService.getWorkingDirectory() + name);
    }

    /**
     * Function that initialised the VDM
     * Should only be called by physicalVehicleBuilder
     */
    public void initialise() {
        if (isInitialised) {
            throw new IllegalStateException("Vehicle dynamics model can only be initialized once.");
        }
        double stopTime = 10.0;
        inputFilter.init(0, stopTime);
        chassis.init(0, stopTime);
        suspension.init(0, stopTime);
        tires.init(0, stopTime);
        brakeSystem.init(0, stopTime);
        driveline.init(0, stopTime);
        steering.init(0, stopTime);

        isInitialised = true;
        exchangeValues();
    }

    /**
     * Function that executes one computation step
     *
     * @param stepSize Size of the computation step
     */
    public void doStep(double stepSize) {
        if (!isInitialised) {
            throw new IllegalStateException("A integration step can only be performed after initialisation.");
        }
        if (needsExchanging) {
            exchangeValues();
        }
        inputFilter.doStep(stepSize);
        chassis.doStep(stepSize);
        suspension.doStep(stepSize);
        tires.doStep(stepSize);
        brakeSystem.doStep(stepSize);
        driveline.doStep(stepSize);
        steering.doStep(stepSize);
        exchangeValues();
    }

    /**
     * Function that sets a parameter value into the corresponding FMU
     *
     * @param name  Name of the parameter value to be set
     * @param value New value to be set
     */
    public void setParameter(String name, double value) {
        if (isInitialised) {
            throw new IllegalStateException("A parameter can only be set before initialisation.");
        }
        switch (name) {
            // Input filter parameters
            case "K_d_road":
                // Input filter initial parameters
            case "slope_d_0":
            case "bank_d_0":
                inputFilter.write(name).with(value);
                break;
            // Chassis parameters
            case "I_x":
            case "I_y":
            case "I_z":
            case "A_f":
            case "C_drag":
            case "f_r":
            case "I_tire":
            case "rho_air":
                // Chassis initial parameters
            case "omega_wheel_1_0":
            case "omega_wheel_2_0":
            case "omega_wheel_3_0":
            case "omega_wheel_4_0":
            case "omega_z_0":
            case "pitch_angle_0":
            case "omega_y_0":
            case "roll_angle_0":
            case "omega_x_0":
            case "v_x_0":
            case "v_y_0":
                chassis.write(name).with(value);
                break;
            // Suspension parameters
            case "Roll_c_f":
            case "Roll_c_r":
            case "K_spring_f":
            case "K_spring_r":
            case "D_shock_f":
            case "D_shock_r":
            case "d_f":
            case "d_r":
            case "L_antiroll_f":
            case "L_antiroll_r":
            case "L_lever_f":
            case "L_lever_r":
            case "G":
            case "d_pitch":
                suspension.write(name).with(value);
                break;
            // Tires parameters
            case "C":
            case "a":
            case "dc":
            case "muv_1":
            case "muv_2":
            case "muv_3":
            case "muv_4":
            case "rlxlen":
                // Tires initial parameters
            case "F_x_1_0":
            case "F_x_2_0":
            case "F_x_3_0":
            case "F_x_4_0":
            case "F_y_1_0":
            case "F_y_2_0":
            case "F_y_3_0":
            case "F_y_4_0":
                tires.write(name).with(value);
                break;
            // Chassis and Suspension parameters
            case "m":
            case "L_1":
            case "L_2":
            case "TW_f":
            case "TW_r":
            case "COG_z":
            case "r_nom":
            case "g":
                chassis.write(name).with(value);
                suspension.write(name).with(value);
                break;
            //Steering parameter
            case "sr":
            case "toe_f":
            case "toe_r":
            case "starm_f":
            case "starm_r":
            case "compfy_f":
            case "compfy_r":
            case "compz_f":
            case "compz_r":
            case "rollsr_f":
            case "rollst_f":
            case "rollst_r":
            case "d_sw":
            case "c_servo":
            case "pinion_r":
            case "fsw":
                steering.write(name).with(value);
                break;
            case "trans_ratio":
            case "i_gear_0":
            case "i_gear_1":
            case "i_gear_2":
            case "i_gear_3":
            case "i_gear_4":
            case "i_gear_5":
            case "i_final":
                driveline.write(name).with(value);
                break;
            case "disc_d_f":
            case "disc_d_r":
            case "cf_pad":
            case "pad_area":
            case "piston_d":
            case "pressure_limit":
                driveline.write(name).with(value);
                break;
            default:
                throw new IllegalArgumentException("Parameter " + name + " does not exist.");
        }
        needsExchanging = true;
    }

    /**
     * Function that sets a input value into the corresponding FMU
     *
     * @param name  Name of the input value to be set
     * @param value New value to be set
     */
    public void setInput(String name, double value) {
        if (!isInitialised) {
            throw new IllegalStateException("A input value can only be set after initialisation.");
        }
        switch (name) {
            // Input filter inputs
            case "bank":
            case "slope":
                // Input filter state variables
            case "slope_d":
            case "bank_d":
            case "delta_sw":
                inputFilter.write(name).with(value);
                break;
            // Chassis inputs
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
                // Chassis state variables
            case "omega_wheel_1":
            case "omega_wheel_2":
            case "omega_wheel_3":
            case "omega_wheel_4":
            case "omega_z":
            case "pitch_angle":
            case "omega_y":
            case "roll_angle":
            case "omega_x":
            case "v_x":
            case "v_y":
                chassis.write(name).with(value);
                break;
            // Tires inputs
            case "mu_1":
            case "mu_2":
            case "mu_3":
            case "mu_4":
                // Tires state variables
            case "F_x_1":
            case "F_x_2":
            case "F_x_3":
            case "F_x_4":
            case "F_y_1":
            case "F_y_2":
            case "F_y_3":
            case "F_y_4":
                tires.write(name).with(value);
                break;
            /* Chassis and tires inputs(now handled by steering)
            case "delta_1":
            case "delta_2":
            case "delta_3":
            case "delta_4":
                chassis.write(name).with(value);
                tires.write(name).with(value);
                break;*/
            // Steering input
            case "omega_sw":
                // Steering state variables
            case "delta_d":
            case "delta_int":
            case "stw_f":
            case "delta_1":
            case "delta_2":
            case "delta_3":
            case "delta_4":
                steering.write(name).with(value);
                break;
            //Driveline inputs
            case "c_input":
            case "t_input":
            case "i":
                // Driveline state variables
            case "engine_tmax":
            case "engine_tmin":
            case "omega_e_int":
            case "omega_e":
            case "i_t":
            case "engine_t_int":
            case "driving_t1":
            case "driving_t2":
            case "driving_t3":
            case "driving_t4":
                driveline.write(name).with(value);
                break;
            //BrakeSystem inputs
            case "b_input":
                // BrakeSystem state variables
            case "brakingtorque_1":
            case "brakingtorque_2":
            case "brakingtorque_3":
            case "brakingtorque_4":
            case "brake_press_f":
            case "brake_press_r":
                brakeSystem.write(name).with(value);
                break;
            default:
                throw new IllegalArgumentException("Input value " + name + " does not exist.");
        }
        needsExchanging = true;
    }

    /**
     * Function that gets a value from the corresponding FMU
     *
     * @param name Name of the value to be returned
     * @return Value with the given name
     */
    public double getValue(String name) {
        if (!isInitialised) {
            throw new IllegalStateException("A value can only be read after initialisation.");
        }
        if (needsExchanging) {
            exchangeValues();
        }
        switch (name) {
            // Input filter parameters
            case "K_d_road":
                //Input filter initial parameter
            case "slope_d_0":
            case "bank_d_0":
                //Input filter inputs
            case "slope":
            case "bank":
                //Input filter variables
            case "slope_d":
            case "bank_d":
                return inputFilter.read(name).asDouble();
            //Chassis parameters
            case "m":
            case "L_1":
            case "L_2":
            case "TW_f":
            case "TW_r":
            case "COG_z":
            case "I_x":
            case "I_y":
            case "I_z":
            case "A_f":
            case "C_drag":
            case "f_r":
            case "I_tire":
            case "r_nom":
            case "rho_air":
            case "g":
                //Chassis Initial parameters
            case "omega_wheel_1_0":
            case "omega_wheel_2_0":
            case "omega_wheel_3_0":
            case "omega_wheel_4_0":
            case "omega_z_0":
            case "pitch_angle_0":
            case "omega_y_0":
            case "roll_angle_0":
            case "omega_x_0":
            case "v_x_0":
            case "v_y_0":
                //Chassis input
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
                //case "delta_1":
                //case "delta_2":
                //case "delta_3":
                //case "delta_4":
                //Chassis variables
            case "a_x":
            case "v_x":
            case "a_y":
            case "v_y":
            case "a_z":
            case "v_z":
            case "z":
            case "alpha_x":
            case "omega_x":
            case "roll_angle":
            case "alpha_y":
            case "omega_y":
            case "pitch_angle":
            case "alpha_z":
            case "omega_z":
            case "alpha_wheel_1":
            case "alpha_wheel_2":
            case "alpha_wheel_3":
            case "alpha_wheel_4":
            case "omega_wheel_1":
            case "omega_wheel_2":
            case "omega_wheel_3":
            case "omega_wheel_4":
                //Chassis outputs
            case "v_x_1":
            case "v_x_2":
            case "v_x_3":
            case "v_x_4":
            case "v_y_1":
            case "v_y_2":
            case "v_y_3":
            case "v_y_4":
            case "v_s_1":
            case "v_s_2":
            case "v_s_3":
            case "v_s_4":
                return chassis.read(name).asDouble();
            //Suspension parameters
            case "Roll_c_f":
            case "Roll_c_r":
            case "K_spring_f":
            case "K_spring_r":
            case "D_shock_f":
            case "D_shock_r":
            case "d_f":
            case "d_r":
            case "L_antiroll_f":
            case "L_antiroll_r":
            case "L_lever_f":
            case "L_lever_r":
            case "G":
            case "d_pitch":
                //"L_1":
                //"L_2":
                //"COG_z":
                //"TW_f":
                //"TW_r":
                //"m":
                //"g":
                //"r_nom":
                //Suspension variables
            case "d_roll":
            case "I_antiroll_f":
            case "K_antiroll_f":
            case "K_roll_f":
            case "I_antiroll_r":
            case "K_antiroll_r":
            case "K_roll_r":
            case "D_roll_f":
            case "D_roll_r":
            case "DeltaF_z_f":
            case "DeltaF_z_r":
            case "K_pitch":
            case "D_pitch":
            case "F_z_1":
            case "F_z_2":
            case "F_z_3":
            case "F_z_4":
                return suspension.read(name).asDouble();
            //Parameters Tires
            case "C":
            case "a":
            case "dc":
            case "muv_1":
            case "muv_2":
            case "muv_3":
            case "muv_4":
            case "rlxlen":
                //Tires initial parameters
            case "F_x_1_0":
            case "F_x_2_0":
            case "F_x_3_0":
            case "F_x_4_0":
            case "F_y_1_0":
            case "F_y_2_0":
            case "F_y_3_0":
            case "F_y_4_0":
                //Tires inputs
            case "mu_1":
            case "mu_2":
            case "mu_3":
            case "mu_4":
                //"delta_1":
                //"delta_2":
                //"delta_3":
                //"delta_4":
                //Tires variables
            case "F_x_1":
            case "F_x_2":
            case "F_x_3":
            case "F_x_4":
            case "F_y_1":
            case "F_y_2":
            case "F_y_3":
            case "F_y_4":
            case "S_x_1":
            case "S_x_2":
            case "S_x_3":
            case "S_x_4":
            case "S_y_1":
            case "S_y_2":
            case "S_y_3":
            case "S_y_4":
            case "S_1":
            case "S_2":
            case "S_3":
            case "S_4":
            case "psi_1":
            case "psi_2":
            case "psi_3":
            case "psi_4":
            case "F_1":
            case "F_2":
            case "F_3":
            case "F_4":
            case "F_x_1_rlx":
            case "F_x_2_rlx":
            case "F_x_3_rlx":
            case "F_x_4_rlx":
            case "F_y_1_rlx":
            case "F_y_2_rlx":
            case "F_y_3_rlx":
            case "F_y_4_rlx":
                return tires.read(name).asDouble();
            //Steering parameters
            case "sr":
            case "toe_f":
            case "toe_r":
            case "starm_f":
            case "starm_r":
            case "compfy_f":
            case "compfy_r":
            case "compz_f":
            case "compz_r":
            case "rollsr_f":
            case "rollst_f":
            case "rollst_r":
            case "d_sw":
            case "c_servo":
            case "pinion_r":
            case "fsw":
                //Steering inputs
            case "delta_sw":
            case "omega_sw":
                //Steering Variables
            case "delta_d":
            case "delta_int":
            case "stw_f":
            case "delta_1":
            case "delta_2":
            case "delta_3":
            case "delta_4":
                return steering.read(name).asDouble();
            //Driveline parameters
            case "trans_ratio":
            case "i_gear_0":
            case "i_gear_1":
            case "i_gear_2":
            case "i_gear_3":
            case "i_gear_4":
            case "i_gear_5":
            case "i_final":
                //Driveline inputs
                //case "v_x:":
                //case "omega_wheel_1":
                //case "omega_wheel_2":
                //Driveline Variables
            case "engine_tmax":
            case "engine_tmin":
            case "omega_e_int":
            case "omega_e":
            case "i_t":
            case "engine_t_int":
            case "driving_t1":
            case "driving_t2":
            case "driving_t3":
            case "driving_t4":
                return driveline.read(name).asDouble();
            //BrakeSystem inputs
            case "b_input":
                //BrakeSystem Variables
            case "brakingtorque_1":
            case "brakingtorque_2":
            case "brakingtorque_3":
            case "brakingtorque_4":
            case "brake_press_f":
            case "brake_press_r":
                return brakeSystem.read(name).asDouble();
            default:
                throw new IllegalArgumentException("Value " + name + " does not exist.");
        }
    }

    /**
     * Function that exchanges the values under the FMUs
     */
    private void exchangeValues() {
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
        tires.write("delta_1").with(steering.read("delta_1").asDouble());
        tires.write("delta_2").with(steering.read("delta_2").asDouble());
        tires.write("delta_3").with(steering.read("delta_3").asDouble());
        tires.write("delta_4").with(steering.read("delta_4").asDouble());
        chassis.write("delta_1").with(steering.read("delta_1").asDouble());
        chassis.write("delta_2").with(steering.read("delta_2").asDouble());
        chassis.write("delta_3").with(steering.read("delta_3").asDouble());
        chassis.write("delta_4").with(steering.read("delta_4").asDouble());
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
        brakeSystem.write("omega_wheel_1").with(chassis.read("omega_wheel_1").asDouble());
        brakeSystem.write("omega_wheel_2").with(chassis.read("omega_wheel_2").asDouble());
        brakeSystem.write("omega_wheel_3").with(chassis.read("omega_wheel_3").asDouble());
        brakeSystem.write("omega_wheel_4").with(chassis.read("omega_wheel_4").asDouble());
        driveline.write("v_x").with(chassis.read("v_x").asDouble());
        driveline.write("omega_wheel_1").with(chassis.read("omega_wheel_1").asDouble());
        driveline.write("omega_wheel_2").with(chassis.read("omega_wheel_2").asDouble());
        steering.write("F_y_1").with(tires.read("F_y_1").asDouble());
        steering.write("F_y_2").with(tires.read("F_y_2").asDouble());
        steering.write("F_y_3").with(tires.read("F_y_3").asDouble());
        steering.write("F_y_4").with(tires.read("F_y_4").asDouble());
        steering.write("mz_1").with(tires.read("mz_1").asDouble());
        steering.write("mz_2").with(tires.read("mz_2").asDouble());
        steering.write("mz_3").with(tires.read("mz_3").asDouble());
        steering.write("mz_4").with(tires.read("mz_4").asDouble());
        steering.write("delta_int").with(inputFilter.read("delta_int").asDouble());
        chassis.write("tau_B_1").with(brakeSystem.read("brakingtorque_1").asDouble());
        chassis.write("tau_B_2").with(brakeSystem.read("brakingtorque_2").asDouble());
        chassis.write("tau_B_3").with(brakeSystem.read("brakingtorque_3").asDouble());
        chassis.write("tau_B_4").with(brakeSystem.read("brakingtorque_4").asDouble());
        chassis.write("tau_D_1").with(driveline.read("driving_t1").asDouble());
        chassis.write("tau_D_2").with(driveline.read("driving_t2").asDouble());
        chassis.write("tau_D_3").with(driveline.read("driving_t3").asDouble());
        chassis.write("tau_D_4").with(driveline.read("driving_t4").asDouble());
        needsExchanging = false;
    }
}
