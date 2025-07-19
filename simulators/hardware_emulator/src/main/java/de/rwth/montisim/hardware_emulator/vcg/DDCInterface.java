/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.vcg;


// public class DDCInterface {
//     public static final int RUNNING_ENTRY = 0;
//     public static final int INTERFACE_TYPE_ENTRY = 1;
//     public static final int TIME_MODE_ENTRY = 2;
//     public static final int DELTA_SEC_ENTRY = 3;
//     public static final int RUN_CYCLE_SWITCH_ENTRY = 4;
//     public static final int EXECUTION_TIME_ENTRY = 5;
//     public static final int SIMULATION_RUNNING_ENTRY = 6;
//     public static final int INPUT_START_ID_ENTRY = 7;
//     public static final int OUTPUT_START_ID_ENTRY = 8;
//     public static final int SLOT_TABLE_START_ENTRY = 7;
//     public static final int DESCRIPTION_SIZE_ENTRY = 8;
//     public static final int INTERFACE_STRING_ENTRY = 9;
//     int reference_id;
//     StringBuilder sb = new StringBuilder();

//     ProgramInterface progInterface;
//     int slotIds[];

//     SSHConsole ssh;

//     public DDCInterface(CommunicationType comm) throws IOException, InterruptedException {
//         if (comm instanceof SSH) {
//             SSH sshComm = (SSH)comm;
//             ssh = new SSHConsole(sshComm.username, sshComm.password, sshComm.host, sshComm.port);
//             if (!sshComm.tunneled)
//                 ssh.connectToVCG(sshComm.vcg);
//             reference_id = sshComm.reference_id;
//         } else throw new IllegalArgumentException("Unimplemented");
//     }

//     public void checkAlive() {
//         // Check if VCG program running:
//         // Set 'running' to false => Wait for 1 with timeout
        
//     }

//     public String getInterfaceType() throws IOException, InterruptedException {
//         return readString(INTERFACE_TYPE_ENTRY);
//     }

//     public String getProgInterface() throws IOException, InterruptedException {
//         int chunks = readInt(DESCRIPTION_SIZE_ENTRY);
//         StringBuilder sb = new StringBuilder();
//         for (int i = 0; i < chunks; ++i){
//             sb.append(readString(INTERFACE_STRING_ENTRY+i));
//         }
//         return sb.toString();
//     }

//     public void setProgramInterface(ProgramInterface progInterface, ProgInterface mode)
//             throws IOException, InterruptedException {
//         this.progInterface = progInterface;
//         int slotCount = progInterface.ports.size();
//         slotIds = new int[slotCount];
//         switch (mode) {
//             case DYNAMIC:
//                 loadSlots(slotCount);
//             break;
//             case BASIC:
//                 // Hardcoded basic interface slots
//                 int inputStart = readInt(INPUT_START_ID_ENTRY);
//                 int outputStart = readInt(OUTPUT_START_ID_ENTRY);
//                 slotIds[0] = inputStart; // true_position
//                 slotIds[1] = inputStart + 2; // true_compass
//                 slotIds[2] = inputStart + 3; // true_velocity
//                 slotIds[3] = inputStart + 4; // trajectory_length
//                 slotIds[4] = inputStart + 5; // trajectory_x
//                 slotIds[5] = inputStart + 15; // trajectory_y
//                 slotIds[6] = inputStart + 25; // steering
//                 slotIds[7] = inputStart + 26; // gas
//                 slotIds[8] = inputStart + 27; // braking

//                 slotIds[9] = outputStart; // set_steering
//                 slotIds[10] = outputStart+1; // set_gas
//                 slotIds[11] = outputStart+2; // set_braking
//             break;
//             default:
//                 throw new IllegalArgumentException("Unsupported case: "+mode);
//         }
//     }

//     protected void loadSlots(int slotCount) throws IOException, InterruptedException {
//         int slotTableStart = readInt(SLOT_TABLE_START_ENTRY);
//         for (int i = 0; i < slotCount; ++i){
//             slotIds[i] = readInt(slotTableStart+i);
//         }
//     }

//     public void setTimeMode(TimeMode mode) throws IOException, InterruptedException {
//         switch(mode) {
//             case MEASURED:
//                 write(TIME_MODE_ENTRY, "measured");
//                 break;
//             case REALTIME:
//                 write(TIME_MODE_ENTRY, "realtime");
//                 break;
//             default:
//                 throw new IllegalArgumentException("Missing TimeMode case");
//         }
//     }

//     public void writeInputs(Object buffer[], double deltaSec) throws IOException, InterruptedException {
//         sb.delete( 0, sb.length() );
//         int i = 0;
//         int lineCount = 0;
//         for (PortInformation port : progInterface.ports) {
//             if (port.direction == PortDirection.INPUT) {
//                 if (buffer[i] != null)
//                     lineCount += getWriteCommand(slotIds[i], port.type, buffer[i]);
//             }
//             ++i;
//         }
//         lineCount += getWriteCommand(DELTA_SEC_ENTRY, deltaSec); // Write time update
//         lineCount += getWriteCommand(RUN_CYCLE_SWITCH_ENTRY, 1); // Turn on cycle switch
//         ssh.sendCommand(sb.toString(), lineCount);
//     }

//     protected void write(int localId, String value) throws IOException, InterruptedException {
//         sb.delete( 0, sb.length() );
//         getWriteCommand(localId, value);
//         ssh.sendCommand(sb.toString(), 1);
//     }

//     protected int getId(int localId) {
//         return reference_id + localId;
//     }

//     protected int readInt(int localId) throws IOException, InterruptedException {
//         sb.delete( 0, sb.length() );
//         getReadCommandInt(localId);
//         ssh.sendCommand(sb.toString(), 2);
//         return Integer.parseInt(ssh.getResponse());
//     }
//     protected String readString(int localId) throws IOException, InterruptedException {
//         sb.delete( 0, sb.length() );
//         getReadCommandInt(localId);
//         ssh.sendCommand(sb.toString(), 2);
//         return ssh.getResponse();
//     }
//     protected double readDouble(int localId) throws IOException, InterruptedException {
//         sb.delete( 0, sb.length() );
//         getReadCommandInt(localId);
//         ssh.sendCommand(sb.toString(), 2);
//         return Double.parseDouble(ssh.getResponse());
//     }

    
//     protected int getWriteCommand(int localId, DataType type, Object o) {
//         if (type instanceof BasicType) {
//             BasicType bt = (BasicType) type;
//             switch (bt.getType()){
//                 case Q:
//                     return getWriteCommand(localId, (Double)o);
//                 case Z:
//                 case N:
//                 case N1:
//                     return getWriteCommand(localId, (Integer)o);
//                 case C:
//                 case VEC2:
//                     Vec2 v2 = (Vec2)o;
//                     int lc = getWriteCommand(localId, v2.x);
//                     return lc + getWriteCommand(localId+1, v2.y);
//                 case VEC3:
//                     Vec3 v3 = (Vec3)o;
//                     int lc2 = getWriteCommand(localId, v3.x);
//                     lc2 += getWriteCommand(localId+1, v3.y);
//                     return lc2 + getWriteCommand(localId+2, v3.z);
//                 case BOOLEAN:
//                     // TODO: check if there is a native command line option for bool
//                     return getWriteCommand(localId, ((Boolean)o ? 1 : 0));
//                 case EMPTY:
//                     // Consider empty as a signal => set target to true
//                     return getWriteCommand(localId, 1);
//                 default: throw new IllegalArgumentException("Missing case");
//             }
//         } else if (type instanceof VectorType) {
//             VectorType vt = (VectorType)type;
//             int lc = 0;
//             Object arr[] = (Object[])o;
//             // TODO Native array types (double[], ...)
//             for (int i = 0; i < vt.getSize(); ++i) {
//                 lc += getWriteCommand(localId+i, vt.getBaseType(), arr[i]);
//             }
//             return lc;
//         } else if (type instanceof MatrixType) {
//             MatrixType mt = (MatrixType)type;
//             int lc = 0;
//             Object arr[][] = (Object[][])o;
//             int pos = 0;
//             for (int i = 0; i < mt.getRowCount(); ++i) {
//                 for (int j = 0; j < mt.getColumnCount(); ++j) {
//                     lc += getWriteCommand(localId+pos, mt.getBaseType(), arr[i][j]);
//                     ++pos;
//                 }
//             }
//             return lc;
//         } else if (type instanceof StructType) {
//             throw new IllegalArgumentException("Unsupported type: "+type.getClass().getName());
//         } else throw new IllegalArgumentException("Unsupported type: "+type.getClass().getName());
//     }

    
//     // Appends to the StringBuilder, includes the line-break
//     // Returns the number of expected response lines
//     protected void getWriteCommand(int localId, String value) {
//         // sb.delete( 0, sb.length() );
//         // return sb.toString();
//         sb.append("ddctool -w -u ");
//         sb.append(getId(localId));
//         sb.append(" -t s -v \"");
//         for (int i = 0; i < value.length(); ++i){
//             char c = value.charAt(i);
//             if (c == '"'){
//                 sb.append("\\\""); // Escape
//             } else sb.append(c);
//         }
//         sb.append("\"\n");
//     }
//     // Appends to the StringBuilder
//     // Returns the number of expected response lines
//     protected int getWriteCommand(int localId, double value) {
//         sb.append("ddctool -w -u ");
//         sb.append(getId(localId));
//         sb.append(" -t d -v ");
//         sb.append(value);
//         return 1;
//     }
//     // Appends to the StringBuilder
//     // Returns the number of expected response lines
//     protected int getWriteCommand(int localId, int value) {
//         sb.append("ddctool -w -u ");
//         sb.append(getId(localId));
//         sb.append(" -t i -v ");
//         sb.append(value);
//         sb.append('\n');
//         return 1;
//     }

//     // Appends to the StringBuilder
//     // Returns the number of expected response lines
//     protected int getReadCommandInt(int localId) {
//         sb.append("ddctool -r -q -u ");
//         sb.append(getId(localId));
//         sb.append('\n');
//         return 2;
//     }
// }
