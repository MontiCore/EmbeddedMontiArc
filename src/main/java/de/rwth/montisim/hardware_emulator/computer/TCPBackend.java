/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.time.Duration;
import java.io.*;
import java.net.*;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.*;

public class TCPBackend implements ComputerBackend {
    public static final String TIME_MODE_REALTIME = "realtime";
    public static final String TIME_MODE_MEASURED = "measured";
    public static final int PACKET_END = 0; // The simulator is closing the connection after
    public static final int PACKET_ERROR = 1; // As payload: An error message
    public static final int PACKET_INIT = 2; // As payload: TimeMode string ("measured" or "realtime")
    public static final int PACKET_INTERFACE = 3; // As payload: TimeMode string ("measured" or "realtime")
    // Payload for INPUT and OUTPUT packets:
    // uint16_t: Port ID (as defined in the DynamicInterface -> Order of appearance)
    // Type dependent payload
    public static final int PACKET_INPUT = 4;
    public static final int PACKET_OUTPUT = 5;
    public static final int PACKET_RUN_CYCLE = 6; // Payload: double: delta_sec
    public static final int PACKET_TIME = 7; // Payload: double: seconds
    public static final int PACKET_REF_ID = 8; // Payload: uint32_t: reference id for the DDC exchange
    public static final int PACKET_PING = 9; // No Payload

    // Small main() to test the protocol
    public static void main(String[] args) throws Exception {
        TypedHardwareEmu.registerTypedHardwareEmu();

        TCP tcp = new TCP();
        tcp.host = "::1";
        tcp.port = 4567;
        TCPBackend client = new TCPBackend(tcp, new MeasuredTime());

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
            Packet p = client.getPacket();
            long end = System.nanoTime();
            // System.out.println("PING: " + Double.toString((end-start)*0.000001)+ "ms
            // (Packet: "+p.id+", payload: "+p.data.length+")");
            System.out.println("PING: " + Long.toString((end - start) / 1000000) + "ms");
        }

        client.sendPacket(PACKET_END);
    }

    Socket cs;
    TCP tcpProperties;
    BufferedReader in;
    DataOutputStream out;
    DataInputStream din;

    ProgramInterface interf;

    public TCPBackend(TCP tcpProperties, TimeModel timeModel) throws Exception {
        this.tcpProperties = tcpProperties;
        
        String timeMode;
        
        if (timeModel instanceof MeasuredTime || timeModel instanceof ConstantTime) {
            timeMode = TIME_MODE_MEASURED;
        } else if (timeModel instanceof Realtime) {
            timeMode = TIME_MODE_REALTIME;
        } else if (timeModel instanceof HardwareTimeModel) {
            throw new IllegalArgumentException("The TCPBackend does not support the HardwareTimeModel.");
        } else throw new IllegalArgumentException("Unknown TimeMode");
        
        cs = new Socket(tcpProperties.host, tcpProperties.port);
        System.out.println("VCG: started TCPClient with host " + tcpProperties.host + " on port "
                + Integer.toString(tcpProperties.port));
        in = new BufferedReader(new InputStreamReader(cs.getInputStream()));
        din = new DataInputStream(cs.getInputStream());
        out = new DataOutputStream(cs.getOutputStream());

        // Send Init packet
        // System.out.println("Sending INIT packet");
        sendPacket(PACKET_INIT, timeMode);
        sendPacket(PACKET_REF_ID, tcpProperties.ref_id);

        // Get "ProgramInterface" packet
        // System.out.println("Waiting for INTERFACE packet");
        Packet p = getPacket();
        switch (p.id) {
            case PACKET_ERROR:
                throw new IllegalStateException("Error on VCG: " + new String(p.data));
            case PACKET_INTERFACE:
                this.interf = Json.instantiateFromJson(new String(p.data), ProgramInterface.class);
                break;
            default:
                throw new IllegalStateException("Unexpected packet with id=" + p.id);
        }
    }
    
    @Override
    public ProgramInterface getInterface() {
        return this.interf;
    }

    private void sendInputPacket(int portId, byte[] bytes) throws IOException {
        out.writeByte(PACKET_INPUT);
        int payloadLength = bytes.length + 2;
        out.writeShort(payloadLength);
        out.writeShort(portId);
        out.write(bytes);
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


    @Override
    public Duration measuredCycle(Object[] portData, double deltaSec) throws IOException {
        // Send inputs
        int i = 0;
        for (PortInformation port : interf.ports) {
            if (port.direction == PortDirection.INPUT && portData[i] != null) {
                // System.out.println("Sending INPUT for "+port.name +": "+portData[i]);
                ByteArrayOutputStream os = new ByteArrayOutputStream();
                DataOutputStream os2 = new DataOutputStream(os);
                port.data_type.toBinary(os2, portData[i]);
                sendInputPacket(i, os.toByteArray());
            }
            ++i;
        }
        // Request execution
        // System.out.println("Sending RUN_CYCLE");
        out.writeByte(PACKET_RUN_CYCLE);
        out.writeShort(8);
        out.writeDouble(deltaSec);
        out.flush();
        // Receive outputs & exec time
        // System.out.println("Waiting for responses");
        Packet p = getPacket();
        while (true) {
            switch (p.id) {
                case PACKET_ERROR:
                    throw new IllegalStateException("Error on VCG: " + new String(p.data));
                case PACKET_TIME:
                    return Time.durationFromSeconds(new DataInputStream(new ByteArrayInputStream(p.data)).readDouble());
                case PACKET_OUTPUT:
                    DataInputStream is = new DataInputStream(new ByteArrayInputStream(p.data));
                    int portId = is.readShort();
                    PortInformation port = interf.ports.elementAt(portId);
                    if (port.direction != PortDirection.OUTPUT)
                        throw new IllegalArgumentException(
                                "Received output packet for port " + port.name + " (but it is an INPUT port)");
                    portData[portId] = port.data_type.fromBinary(is);
                    break;
                default:
                    throw new IllegalArgumentException("Unexpected packet with id=" + p.id);
            }
            p = getPacket();
        }
    }

    @Override
    protected void finalize() throws Throwable {
        if (cs != null)
            cs.close();
    }

    static class Packet {
        int id;
        byte[] data;

        Packet(int id, byte[] data) {
            this.id = id;
            this.data = data;
        }
    }

    @Override
    public void destroy() {
        try {
            sendPacket(PACKET_END);
            if (cs != null) cs.close();
            cs = null;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
