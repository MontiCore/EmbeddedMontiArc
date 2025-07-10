/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;

/**
 * The associated Java object must be an Object array with two entries:
 * - The Target/Sender IP (depending wether before or after the message passed the Network). (String)
 * - The Payload: Associated object with 'payloadType'.
 */
@Typed(SimplePacketType.TYPE)
public class SimplePacketType extends DataType {
    public static final String TYPE = "simple_packet";
    public static final int HEADER_SIZE = 16; // TODO

    private DataType payloadType;

    public SimplePacketType(DataType payloadType) {
        this.payloadType = payloadType;
    }

    private SimplePacketType() {
    }

    public DataType getPayloadType() {
        return payloadType;
    }

    @Override
    public int getDataSize(Object o) {
        Object[] arr = (Object[]) o;
        return HEADER_SIZE + payloadType.getDataSize(arr[1]);
    }

    @Override
    public int hashCode() {
        final int prime = 17;
        int result = 7;
        result = prime * result + ((payloadType == null) ? 0 : payloadType.hashCode());
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        SimplePacketType a = ((SimplePacketType) o);
        return this.payloadType.equals(a.payloadType);
    }

    @Override
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        Object packet[] = (Object[]) o;
        j.startArray();
        Json.toJson(j, packet[0], context);
        payloadType.toJson(j, packet[1], context);
        j.endArray();
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        Object res[] = new Object[2];
        Iterator<ValueType> it = j.streamArray().iterator();
        if (!it.hasNext()) throw new ParsingException("Missing 'Address' entry in SimplePacket.");
        it.next();
        res[0] = Json.instantiateFromJson(j, String.class, context);
        if (!it.hasNext()) throw new ParsingException("Missing 'Payload' entry in SimplePacket.");
        it.next();
        res[1] = payloadType.fromJson(j, context);
        if (it.hasNext()) throw new ParsingException("Unexpected entry in SimplePacket.");
        return res;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        Object packet[] = (Object[]) o;
        String addr = (String)packet[0];
        os.write(addr.getBytes());
        os.writeByte(0); // Termination char
        payloadType.toBinary(os, packet[1]);
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        Object res[] = new Object[2];
        StringBuilder sb = new StringBuilder();
        byte c;
        do {
            c = is.readByte();
            if (c == 0) break;
            sb.append((char)c);
        } while (true);
        res[0] = sb.toString();
        res[1] = payloadType.fromBinary(is);
        return res;
    }

    @Override
    public List<String> toString(Object o) {
        Object packet[] = (Object[]) o;
        ArrayList<String> res = new ArrayList<>();
        res.add("SimplePacket: addr="+((String) packet[0])+" payload=");
        res.addAll(payloadType.toString(packet[1]));
        return res;
    }

    @Override
    public String toString() {
        return "SimplePacket(" + payloadType + ")";
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
    }
    
}
