/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.time.Duration;
import java.io.*;
import java.net.*;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.hardware_emulator.computer.Computer.SocketQueues;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.*;

public class TCPBackend implements ComputerBackend {
    // SEE https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/dev-docs/concepts/TCP-protocol

    
    static class Packet {
        int id;
        byte[] data;

        Packet(int id, byte[] data) {
            this.id = id;
            this.data = data;
        }
    }

    // TODO: more robust protocol (ex: FIRST packet exchange: VERSION of this protocol)
    public static final String TIME_MODE_REALTIME = "realtime";
    public static final String TIME_MODE_MEASURED = "measured";
    public static final String TIME_MODE_HARDWARE = "hardware";
    public static final int PACKET_END = 0; // The simulator is closing the connection after
    public static final int PACKET_ERROR = 1; // As payload: An error message
    public static final int PACKET_INIT = 2; // As payload: TimeMode string ("measured" or "realtime")
    public static final int PACKET_INTERFACE = 3; // As payload: TimeMode string ("measured" or "realtime")
    // Payload for INPUT and OUTPUT packets:
    // uint16_t: Port ID (as defined in the DynamicInterface -> Order of appearance)
    // Type dependent payload
    public static final int PACKET_INPUT_BINARY = 4;
    public static final int PACKET_OUTPUT_BINARY = 5;
    public static final int PACKET_RUN_CYCLE = 6; // Payload: double: delta_sec
    public static final int PACKET_TIME = 7; // Payload: double: seconds
    public static final int PACKET_REF_ID = 8; // Payload: uint32_t: reference id for the DDC exchange
    public static final int PACKET_PING = 9; // No Payload
    public static final int PACKET_REQUEST_CONFIG = 10; // No Payload
    public static final int PACKET_CONFIG = 11; // Payload: JSON string, response to PACKET_REQUEST_CONFIG
    public static final int PACKET_INPUT_JSON = 13;
    public static final int PACKET_OUTPUT_JSON = 14;
    public static final int PACKET_SUSPEND = 15; // No Payload
    public static final int PACKET_EMU_ID = 16; // Payload: emulator token
    public static final int PACKET_RECONNECT = 17; // Payload: emulator token


    Socket cs;
    TCP tcpProperties;
    ComputerProperties properties;
    BufferedReader in;
    DataOutputStream out;
    DataInputStream din;

    ProgramInterface interf;

    transient JsonWriter writer = new JsonWriter(false);
    transient JsonTraverser traverser = new JsonTraverser();

    public TCPBackend(TCP tcpProperties, ComputerProperties properties, TimeModel timeModel) throws Exception {
        this.properties = properties;
        this.tcpProperties = tcpProperties;
        
        String timeMode;
        
        if (timeModel instanceof MeasuredTime || timeModel instanceof ConstantTime) {
            timeMode = TIME_MODE_MEASURED;
        } else if (timeModel instanceof Realtime) {
            timeMode = TIME_MODE_REALTIME;
        } else if (timeModel instanceof HardwareTimeModel) {
            timeMode = TIME_MODE_HARDWARE;
            //throw new IllegalArgumentException("The TCPBackend does not support the HardwareTimeModel.");
        } else throw new IllegalArgumentException("Unknown TimeMode");
        
        cs = new Socket(tcpProperties.host, tcpProperties.port);
        System.out.println("Computer: started TCPBackend with host " + tcpProperties.host + " on port "
                + Integer.toString(tcpProperties.port));
        in = new BufferedReader(new InputStreamReader(cs.getInputStream()));
        din = new DataInputStream(cs.getInputStream());
        out = new DataOutputStream(cs.getOutputStream());

        if (tcpProperties.emu_id != 0) {
            // Special case: Reconnect to remote hardware_emulator
            sendPacketLong(PACKET_RECONNECT, tcpProperties.emu_id);
        } else {

            // Send Init packet
            // System.out.println("Sending INIT packet");
            sendPacket(PACKET_INIT, timeMode);
            sendPacket(PACKET_REF_ID, tcpProperties.ref_id);

        }

        
        // Get "ProgramInterface" packet
        // System.out.println("Waiting for INTERFACE packet");
        boolean repeat;
        do {
            repeat = false;
            Packet p = getPacket();
            switch (p.id) {
                case PACKET_ERROR:
                    throw new IllegalStateException("Error on Remote Computer: " + new String(p.data));
                case PACKET_INTERFACE:
                    this.interf = Json.instantiateFromJson(new String(p.data), ProgramInterface.class);
                    break;
                case PACKET_REQUEST_CONFIG: // The remote is a hardware_emulator -> Respond with the configuration
                    sendPacket(PACKET_CONFIG, Json.toJson(properties));
                    repeat = true;
                    break;
                default:
                    throw new IllegalStateException("Unexpected packet with id=" + p.id);
            }
        } while (repeat);
    }

    
    @Override
    public ProgramInterface getInterface() {
        return this.interf;
    }



    @Override
    public Duration measuredCycle(Object[] portData, double deltaSec) throws Exception {
        // Send inputs
        int i = 0;
        for (PortInformation port : interf.ports) {
            if (port.isInput()) {
                if (port.port_type == PortType.SOCKET) {
                    SocketQueues sq = (SocketQueues)portData[i];
                    for (Object o : sq.in) {
                        // System.out.println("Sending INPUT for "+port.name +": "+portData[i]);
                        if (properties.json_data_exchange) {
                            writer.init();
                            port.data_type.toJson(writer, o, null);
                            sendInputPacket(i, writer.getString());
                        } else {
                            ByteArrayOutputStream os = new ByteArrayOutputStream();
                            DataOutputStream os2 = new DataOutputStream(os);
                            port.data_type.toBinary(os2, o);
                            sendInputPacket(i, os.toByteArray());
                        }
                    }
                    sq.in.clear();
                } else {
                    if (portData[i] != null) {
                        // System.out.println("Sending INPUT for "+port.name +": "+portData[i]);
                        if (properties.json_data_exchange) {
                            writer.init();
                            port.data_type.toJson(writer, portData[i], null);
                            sendInputPacket(i, writer.getString());
                        } else {
                            ByteArrayOutputStream os = new ByteArrayOutputStream();
                            DataOutputStream os2 = new DataOutputStream(os);
                            port.data_type.toBinary(os2, portData[i]);
                            sendInputPacket(i, os.toByteArray());
                        }
                        
                    }
                }
            }
            ++i;
        }
        // Request execution
        // System.out.println("Sending RUN_CYCLE");
        sendPacket(PACKET_RUN_CYCLE, deltaSec);
        // Receive outputs & exec time
        // System.out.println("Waiting for responses");
        i = 0;
        for (PortInformation p : interf.ports) {
            if (p.isOutput()) {
                if (p.port_type == PortType.SOCKET) {
                    SocketQueues sq = (SocketQueues)portData[i];
                    sq.out.clear();
                }
            }
            ++i;
        }

        while (true) {
            Packet p = getPacket();
            switch (p.id) {
                case PACKET_ERROR:
                    throw new IllegalStateException("Error on VCG: " + new String(p.data));
                case PACKET_TIME:
                    return Time.durationFromSeconds(new DataInputStream(new ByteArrayInputStream(p.data)).readDouble());
                case PACKET_OUTPUT_BINARY: {
                    DataInputStream is = new DataInputStream(new ByteArrayInputStream(p.data));
                    int portId = is.readShort();
                    PortInformation port = interf.ports.elementAt(portId);
                    if (!port.isOutput())
                        throw new IllegalArgumentException(
                                "Received output packet for port " + port.name + " (but it is an INPUT port)");
                    Object res = port.data_type.fromBinary(is);
                    if (port.port_type == PortType.SOCKET) {
                        SocketQueues sq = (SocketQueues)portData[portId];
                        sq.out.add(res);
                    } else {
                        portData[portId] = res;
                    }
                } break;
                case PACKET_OUTPUT_JSON: {
                    DataInputStream is = new DataInputStream(new ByteArrayInputStream(p.data));
                    int portId = is.readShort();
                    PortInformation port = interf.ports.elementAt(portId);
                    if (!port.isOutput())
                        throw new IllegalArgumentException(
                                "Received output packet for port " + port.name + " (but it is an INPUT port)");
                    String data = new String(p.data, 2, p.data.length - 2);
                    traverser.init(data);
                    Object res = port.data_type.fromJson(traverser, null);
                    if (port.port_type == PortType.SOCKET) {
                        SocketQueues sq = (SocketQueues)portData[portId];
                        sq.out.add(res);
                    } else {
                        portData[portId] = res;
                    }
                } break;
                default:
                    throw new IllegalArgumentException("Unexpected packet with id=" + p.id);
            }
        }
    }

    

    @Override
    public void destroy() {
        try {
            sendPacket(PACKET_END);
            closeConnection();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void pop() {
        try {
            suspend();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    
    public void suspend() throws IOException {
        sendPacket(PACKET_SUSPEND);
        // Wait for PACKET_EMU_ID
        Packet p = getPacket();
        switch (p.id) {
            case PACKET_ERROR:
                throw new IllegalStateException("Error on Remote Computer: " + new String(p.data));
            case PACKET_EMU_ID: {
                DataInputStream is = new DataInputStream(new ByteArrayInputStream(p.data));
                this.tcpProperties.emu_id = is.readLong();
            } break;
            default:
                throw new IllegalStateException("Unexpected packet with id=" + p.id);
        }
        closeConnection();
    }
    

    protected void closeConnection() throws IOException {
        if (cs != null) cs.close();
        cs = null;
    }

    @Override
    protected void finalize() throws Throwable {
        closeConnection();
    }

    
    private Packet getPacket() throws IOException {
        int id = din.readByte();
        int length = din.readShort();
        // byte[] data = din.readNBytes(length);
        // Apparently readNBytes() is only in new versions => Emulate
        byte[] data = new byte[length];
        for (int i = 0; i < length; ++i) {
            data[i] = din.readByte();
        }
        // System.out.println("Received packet: id="+id + ", length="+length+",
        // bytes="+new String(data));
        // System.out.println("Received packet: id="+id + ", length="+length);
        return new Packet(id, data);
    }

    private void sendInputPacket(int portId, byte[] bytes) throws IOException {
        out.writeByte(PACKET_INPUT_BINARY);
        int payloadLength = bytes.length + 2;
        out.writeShort(payloadLength);
        out.writeShort(portId);
        out.write(bytes);
        out.flush();
    }
    private void sendInputPacket(int portId, String jsonPayload) throws IOException {
        out.writeByte(PACKET_INPUT_JSON);
        int payloadLength = jsonPayload.length() + 3;
        out.writeShort(payloadLength);
        out.writeShort(portId);
        out.write(jsonPayload.getBytes());
        out.writeByte('\0');
        out.flush();
    }

    private void sendPacket(int packetId, byte[] bytes) throws IOException {
        out.writeByte(packetId);
        int payloadLength = bytes.length;
        out.writeShort(payloadLength);
        out.write(bytes);
        out.flush();
    }

    private void sendPacket(int packetId, int val) throws IOException {
        out.writeByte(packetId);
        out.writeShort(4);
        out.writeInt(val);
        out.flush();
    }
    private void sendPacketLong(int packetId, long val) throws IOException {
        out.writeByte(packetId);
        out.writeShort(8);
        out.writeLong(val);
        out.flush();
    }
    private void sendPacket(int packetId, double val) throws IOException {
        out.writeByte(packetId);
        out.writeShort(8);
        out.writeDouble(val);
        out.flush();
    }

    private void sendPacket(int packetId, String str) throws IOException {
        out.writeByte(packetId);
        int payloadLength = str.length() + 1;
        out.writeShort(payloadLength);
        out.write(str.getBytes());
        out.writeByte('\0');
        out.flush();
    }

    // No payload
    private void sendPacket(int packetId) throws IOException {
        out.writeByte(packetId);
        out.writeShort(1);
        out.writeByte(0); // Pad to 4 bytes or the message doesn't seem to be sent directly
        out.flush();
    }

    
    // Small main() to test the protocol
    public static void main(String[] args) throws Exception {
        TypedHardwareEmu.registerTypedHardwareEmu();

        TCP tcp = new TCP();
        tcp.host = "::1";
        tcp.port = 4567;
        TCPBackend client = new TCPBackend(tcp, null, new MeasuredTime());

        ProgramInterface interf = client.getInterface();

        Object[] data = new Object[interf.ports.size()];
        data[0] = 0.0;
        data[1] = new Vec2(0, 0);
        data[2] = 0.0;
        data[3] = 3;
        data[4] = new double[] { 1.0, 2.0, 3.0 };
        data[5] = new double[] { 0.0, 1.0, 1.0 };
        data[6] = 0.0;
        data[7] = 0.0;
        data[8] = 0.0;
        System.out.println("Initialized VCG");
        Duration dur = client.measuredCycle(data, 0.1);
        System.out.println("Ran cycle in " + dur + " secs.");
        System.out.println("steering=" + data[9] + " gas=" + data[10] + " braking=" + data[11]);

        // Test PING
        for (int i = 0; i < 10; ++i) {
            long start = System.nanoTime();
            client.sendPacket(PACKET_PING);
            // client.sendPacket(PACKET_PING, 42);
            client.getPacket();
            long end = System.nanoTime();
            // System.out.println("PING: " + Double.toString((end-start)*0.000001)+ "ms
            // (Packet: "+p.id+", payload: "+p.data.length+")");
            System.out.println("PING: " + Long.toString((end - start) / 1000000) + "ms");
        }

        client.sendPacket(PACKET_END);
    }


}
