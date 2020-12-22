package de.rwth.montisim.simulation.eecomponents.simple_network;

public class IPV6Address {
    public static final IPV6Address BROADCAST_ADDR = new IPV6Address("ff02::1");
    public String addr;
    public IPV6Address(String addr) {
        this.addr = addr;
    }
}
