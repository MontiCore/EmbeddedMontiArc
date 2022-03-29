package de.gdl.rl.ros;

import de.isse.jros.RosNode;
import de.isse.jros.RosNode.Publishing;

import de.isse.jros.fields.RBoolean;
import de.isse.jros.fields.RInteger;
//import de.isse.jros.fields.RArray;

import de.isse.jros.messages.StdMsgs;

import java.net.*;
import de.isse.jros.types.*;

import java.util.HashMap;

public class RosConnector 
{

    private boolean WITH_DEBUG_OUTPUT = false;

    private RosNode node;

    HashMap<String, Publishing> publishers = new HashMap<String, Publishing>();

    private RosConnectorSubscriber subscriber = null;

    public RosConnector(RosConnectorSubscriber subscriber) {

        this.subscriber = subscriber;

        try {
            
            this.node = new RosNode("gdlEnv", "http://" + InetAddress.getLocalHost().getHostAddress() + ":11311", InetAddress.getLocalHost().getHostAddress());
            System.out.println("ROSGDLConnector inited.");

        } catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }

    }

    public boolean gotPublisherForTopic(String topic) {
   
        if (this.node != null && this.node.getConnectionTopics() != null) {
            for (String t : this.node.getConnectionTopics()) {
                String[] splitted = t.split(" ");
                if (splitted.length > 2) {
                    if (splitted[2].equals(topic)) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }

    public void addIntSubscriber(String topic) {
        try {
            this.node.subscribe(topic, new ROSstruct("std_msgs/Int32").withInt32("data") , new RosNode.Subscriber() {
                @Override
                public void received(byte[] message) {
                    RInteger value = RInteger.uint32();
                    int fvalue = (int) value.read(message);
                    RosConnector.this.subscriber.receivedData(topic, fvalue);
                }
            });
        } catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }
    }

    public void addBoolSubscriber(String topic) {
        try {
            this.node.subscribe(topic, new ROSstruct("std_msgs/Bool").withBool("data"), new RosNode.Subscriber() {

                @Override
                public void received(byte[] message) {
                    boolean value = RBoolean.bool().read(message);
                    RosConnector.this.subscriber.receivedData(topic, value);
                }  
            });
        } catch (Exception e) {

        }

    }

    public void addFloatArrPublisher(String topic) {
        try {
            Publishing publisher = this.node.publish(topic, StdMsgs.Float32MultiArray(), true);
            publishers.put(topic, publisher);
        } catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }
    }

    public void addFloatPublisher(String topic) {
        try {
            Publishing publisher = this.node.publish(topic, new ROSstruct("std_msgs/Float32").withFloat32("data"), true);
            publishers.put(topic, publisher);
        } catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }
    }

    public void addBoolPublisher(String topic) {
        try {
            Publishing publisher = this.node.publish(topic, new ROSstruct("std_msgs/Bool").withBool("data"), true);
            publishers.put(topic, publisher);
        } catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }
    }


    public void publish(String topic, float[] value) {
        if (this.WITH_DEBUG_OUTPUT) System.out.println("publish on '" + topic + "'");

        byte[] msg = new byte[1000 + value.length * 4]; 
        
        StdMsgs.Float32MultiArray multiarray = new StdMsgs.Float32MultiArray();
        multiarray.layout.data_offset.write(msg, 0);
        multiarray.layout.dim.resize(msg, 0);
        multiarray.data.resize(msg, value.length);

        for (int i = 0; i < value.length; i++) {
            multiarray.data.get(i).write(msg, value[i]);
        }
        
        if (this.publishers.containsKey(topic)) {
            try {
                this.publishers.get(topic).send(msg);
            } catch (Exception e) {
                if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
            }
        }
    }

    public void publish(String topic, float value) {
 
        byte[] msg = new byte[1000];
        StdMsgs.Float32 obj = new StdMsgs.Float32();
        obj.data.write(msg, value);
        if (this.publishers.containsKey(topic)) {
            try {
                this.publishers.get(topic).send(msg);
            } catch (Exception e) {
                if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
            }
        }
        
    }

    public void publish(String topic, boolean value) {
        byte[] msg = new byte[1000];
        StdMsgs.Bool obj = new StdMsgs.Bool();
        obj.data.write(msg, value);
        if (this.publishers.containsKey(topic)) {
            try {
                this.publishers.get(topic).send(msg);
            } catch (Exception e) {
                if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
            }
        }
    }

}