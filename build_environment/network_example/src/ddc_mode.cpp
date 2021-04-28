#include "ddc_mode.h"
#include <iostream>
#include <unistd.h>
#include <chrono>

#include "program_adapter.h"
#include "ddc/cppwrapper/ddcwrapper.h"
#include "ddc_interface.h"
#define name(e) #e << " (" << e << ')'

using namespace dsa::cpp::ddc;
using namespace std;

int write_header(int ref_id, DdcWrapper &ddc);
int write_slot_table(int ref_id, DdcWrapper &ddc);
int write_defaults(int ref_id, DdcWrapper &ddc);
void read_ddc_inputs(int ref_id, DdcWrapper &ddc, Autopilot &ap);
void write_ddc_outputs(int ref_id, DdcWrapper &ddc, Autopilot &ap);

int slot_table_offset;
int slots_start;


/*
    DDC MODE
*/

int ddc_mode(char *reference_id, bool &running) {

    // Check reference id
    int ref_id = strtol(reference_id, nullptr, 16);
    if (ref_id < MIN_DDC_ID) {
        std::cerr << "The reference id "<< std::hex << ref_id << " should be above " << MIN_DDC_ID << std::endl;
        return -1;
    }

    // Load the DDC
    DdcWrapper ddc;
    if (!ddc.isDdcOpen()){
        std::cerr << "Could not open DDC" << std::endl;
        return -1;
    }

    Autopilot autopilot;
    autopilot.init();

    // Clear DDC range or else previous entries with different types can mess up the writing
    // Make sure you cover the entire range (here 70 entries)
    for (int i = 0; i < 70; ++i) ddc.saveDataNoData(ref_id+i);

    // Write "DDC header"
    write_header(ref_id, ddc);
    // Write slot table
    write_slot_table(ref_id, ddc);
    // Write default ddc values
    write_defaults(ref_id, ddc);

    cout << "Written DDC-interface to DDC" << endl;
    cout << "Waiting for Simulation..." << endl;

    // Measured / Realtime:
    //   Wait for "Run cycle" / "Simulation Running"
    //   Read DDC inputs
    //   Run cycle
    //   Write DDC outputs
    //   Write exec time (if Measured mode)
    //   Set Run Cycle Switch off (if measured mode)

    bool first = true;

    // Running is the flag controlled by the termination signal (ctrl+C)
    while (running) {
        bool run_cycle_switch = false;
        bool sim_running = false;
        ddc.readDataBool(ref_id + ID_RUN_CYCLE_SWITCH, run_cycle_switch);
        ddc.readDataBool(ref_id + ID_SIM_RUNNING, sim_running);
    
        if ( sim_running || run_cycle_switch) {

            if (first) {
                cout << "Running..." << endl;
                first = false;
            }

            // Read DDC inputs
            read_ddc_inputs(ref_id, ddc, autopilot);

            // Run cycle
            double delta_secs = 0;
            ddc.readDataDouble(ref_id+ID_DELTA_SEC, delta_secs);
            //cout << "delta_sec: "<< delta_secs << endl;

            auto t1 = HRClock::now();
            autopilot.execute(delta_secs);
            auto t2 = HRClock::now();

            // Write DDC outputs
            write_ddc_outputs(ref_id, ddc, autopilot);

            if (run_cycle_switch) { // For Measured mode
                // Write exec time
                duration exec_time = t2 - t1;
                ddc.saveDataDouble(ref_id+ID_EXEC_TIME, exec_time.count());
                // Write Run cycle (to false) => DDC server knows that outputs have been updated
                ddc.saveDataBool(ref_id+ID_RUN_CYCLE_SWITCH, false);
                //cout << "exec_time: " << exec_time.count() << 's' << endl;
            } else {
                // TODO output update flag ?
            }


        } else {
            // Wait for the Run Flags
            usleep(10*1000);
            ddc.saveDataBool(ref_id + ID_RUNNING, true);
        }
    }

    return 0;
}


void read_ddc_inputs(int ref_id, DdcWrapper &ddc, Autopilot &ap) {
    ddc.readDataDouble(ref_id + slots_start, ap.true_velocity); // true_velocity
    ddc.readDataDouble(ref_id + slots_start+1, ap.true_position.x); // true_position
    ddc.readDataDouble(ref_id + slots_start+2, ap.true_position.y);
    ddc.readDataDouble(ref_id + slots_start+3, ap.true_compass); // true_compass
    int traj_length = 0;
    ddc.readDataInt32(ref_id + slots_start+4, traj_length); // trajectory_length
    ap.trajectory_x__size = traj_length;
    ap.trajectory_y__size = traj_length;
    // trajectory_x
    for(int i = 0; i < 10; ++i)
        ddc.readDataDouble(ref_id + slots_start+5+i, ap.trajectory_x[i]);
    // trajectory_y
    for(int i = 0; i < 10; ++i)
        ddc.readDataDouble(ref_id + slots_start+15+i, ap.trajectory_y[i]);
}

void write_ddc_outputs(int ref_id, DdcWrapper &ddc, Autopilot &ap) {
    ddc.saveDataDouble(ref_id + slots_start+28, ap.set_steering); // set_steering
    ddc.saveDataDouble(ref_id + slots_start+29, ap.set_gas); // set_gas
    ddc.saveDataDouble(ref_id + slots_start+30, ap.set_braking); // set_braking
}



int write_slot_table(int ref_id, DdcWrapper &ddc) {
    slots_start = slot_table_offset + PROGRAM_PORT_COUNT + 5;

    ddc.saveDataInt32(ref_id + slot_table_offset, slots_start); // true_velocity
    ddc.saveDataInt32(ref_id + slot_table_offset + 1, slots_start + 1); // true_position
    ddc.saveDataInt32(ref_id + slot_table_offset + 2, slots_start + 3); // true_compass
    ddc.saveDataInt32(ref_id + slot_table_offset + 3, slots_start + 4); // trajectory_length
    ddc.saveDataInt32(ref_id + slot_table_offset + 4, slots_start + 5); // trajectory_x
    ddc.saveDataInt32(ref_id + slot_table_offset + 5, slots_start + 15); // trajectory_y
    ddc.saveDataInt32(ref_id + slot_table_offset + 6, slots_start + 25); // steering
    ddc.saveDataInt32(ref_id + slot_table_offset + 7, slots_start + 26); // gas
    ddc.saveDataInt32(ref_id + slot_table_offset + 8, slots_start + 27); // braking
    ddc.saveDataInt32(ref_id + slot_table_offset + 9, slots_start + 28); // set_steering
    ddc.saveDataInt32(ref_id + slot_table_offset + 10, slots_start + 29); // set_gas
    ddc.saveDataInt32(ref_id + slot_table_offset + 11, slots_start + 30); // set_braking
}


int write_defaults(int ref_id, DdcWrapper &ddc) {
    ddc.saveDataDouble(ref_id + slots_start, 0); // true_velocity
    // true_position
    ddc.saveDataDouble(ref_id + slots_start+1, 0);
    ddc.saveDataDouble(ref_id + slots_start+2, 0);
    ddc.saveDataDouble(ref_id + slots_start+3, 0); // true_compass
    ddc.saveDataInt32(ref_id + slots_start+4, 0); // trajectory_length
    // trajectory_x & trajectory_y
    for(int i = ref_id + slots_start+5; i < ref_id + slots_start+25; ++i)
        ddc.saveDataDouble(i, 0);
    ddc.saveDataDouble(ref_id + slots_start+25, 0); // steering
    ddc.saveDataDouble(ref_id + slots_start+26, 0); // gas
    ddc.saveDataDouble(ref_id + slots_start+27, 0); // braking
    ddc.saveDataDouble(ref_id + slots_start+28, 0); // set_steering
    ddc.saveDataDouble(ref_id + slots_start+29, 0); // set_gas
    ddc.saveDataDouble(ref_id + slots_start+30, 0); // set_braking
}


int write_header(int ref_id, DdcWrapper &ddc) {
    ddc.saveDataBool(ref_id + ID_RUNNING, true);
    ddc.saveDataString(ref_id + ID_INTERFACE_TYPE, PROGRAM_INTERFACE_TYPE);
    ddc.saveDataString(ref_id + ID_TIME_MODE, "");
    ddc.saveDataBool(ref_id + ID_RUN_CYCLE_SWITCH, false);
    ddc.saveDataDouble(ref_id + ID_DELTA_SEC, 0);
    ddc.saveDataDouble(ref_id + ID_EXEC_TIME, 0);
    ddc.saveDataBool(ref_id + ID_SIM_RUNNING, false);

    // Write Dynamic Interface
    std::string prog_inf(PROGRAM_INTERFACE); // Here is it the same as the "Basic" interface, but described using the "DynamicInterface" method.
    const int size = prog_inf.size();
    // Split the string into chunks
    int chunks = ((size - 1) / DDC_MAX_LENGTH) + 1;

    ddc.saveDataInt32(ref_id + ID_INTERFACE_STRING_COUNT, chunks);
    for (int i = 0, pos = 0; i < chunks; ++i, pos += DDC_MAX_LENGTH) {
        auto sub = prog_inf.substr(pos, std::min(DDC_MAX_LENGTH, size-pos));
        ddc.saveDataString(ref_id + ID_INTERFACE_STRING + i, sub);
    }
    
    slot_table_offset = ID_INTERFACE_STRING + chunks + 2;

    ddc.saveDataInt32(ref_id + ID_SLOT_TABLE_START, slot_table_offset);

    return 0;
}
