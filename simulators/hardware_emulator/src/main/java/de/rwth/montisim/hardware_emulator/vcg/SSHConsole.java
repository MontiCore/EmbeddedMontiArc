/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.vcg;

import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

// import org.apache.sshd.client.SshClient;
// import org.apache.sshd.client.channel.ClientChannel;
// import org.apache.sshd.client.session.ClientSession;
// import org.apache.sshd.common.channel.Channel;

// /*
//     Discontinued: This way of interacting with the hardware is REALLY slow (roundtrips in the 100ms range).
//     Plus there is the need to continuously pull for some kind of interaction.
// */
// public class SSHConsole {
//     static final long TIMEOUT_MILLISECONDS = 1000;
//     SshClient client;
//     ClientSession session;
//     ClientChannel channel;
//     boolean linesReceived = false;
//     List<String> lines;

//     ShellOutput shellOutput = new ShellOutput();
//     OutputStream shellInput;

    

//     public SSHConsole(String username, String password, String host, int port) throws IOException, InterruptedException {
//         // Setup Client
//         client = SshClient.setUpDefaultClient();
//         client.start();

//         // Setup SSH shell
//         session = client.connect(username, host, port)
//                 .verify(TIMEOUT_MILLISECONDS, TimeUnit.MILLISECONDS).getSession();
//         session.addPasswordIdentity(password);
//         //connect(HostConfigEntry hostConfig)
//         session.auth().verify(TIMEOUT_MILLISECONDS, TimeUnit.MILLISECONDS);

//         channel = session.createChannel(Channel.CHANNEL_SHELL);
//         channel.setOut(shellOutput);
        
//         channel.open().verify(TIMEOUT_MILLISECONDS, TimeUnit.MILLISECONDS);
//         shellOutput.expectLines(3);
//         shellInput = channel.getInvertedIn();
//     }

//     public void connectToVCG(String vcg) throws IOException, InterruptedException {
//         sendCommand("ssh "+vcg, 2);
//     }

//     public void sendCommand(String command, int expectedResponseLines) throws IOException, InterruptedException {
//         shellInput.write((command).getBytes());
//         shellInput.flush();
//         if(expectedResponseLines > 0)
//             shellOutput.expectLines(expectedResponseLines);
//     }


//     public double readDouble(String id) throws IOException, InterruptedException {
//         sendCommand("ddctool -r -q -u " + id, 2);
//         if (!linesReceived || lines == null || lines.size() < 2) throw new IllegalArgumentException("Did not receive response");
//         return Double.parseDouble(lines.get(1));
//     }

//     public String getResponse(){
//         if (!linesReceived || lines == null || lines.size() < 2) throw new IllegalArgumentException("Did not receive response");
//         return lines.get(1);
//     }

//     public void disconnect() {
//         if (channel != null){
//             channel.close(false);
//             channel = null;
//         }
//         if (client != null) {
//             client.stop();
//             client = null;
//         }
//     }

//     public void finalize() {
//         disconnect();
//     }



//     class ShellOutput extends OutputStream {
//         StringBuilder str = new StringBuilder();
//         int expectedLines = 0;
//         List<String> l = new ArrayList<>();
//         long start_time = 0;

//         @Override
//         public void write(int b) throws IOException {
//             if (b == '\n') {
//                 String res = str.toString();
//                 str.setLength(0);
//                 // HACK?
//                 System.out.println("[R] " + res);
//                 if (res.startsWith("ddctool")) return; // Ignore sent commands


//                 l.add(res);

//                 if (expectedLines <= 0){
//                     throw new IllegalArgumentException("Unexpected Line from SSH: "+res);
//                 }
//                 --expectedLines;
//                 if (expectedLines <= 0){
//                     receivedLines();
//                 }
                
//             } else
//                 str.append((char) b);
//         }

//         void expectLines(int count) throws InterruptedException {
//             start_time = System.currentTimeMillis();
//             synchronized(this) {
//                 linesReceived = false;
//                 lines = null;
//                 expectedLines = count;
//                 l = new ArrayList<>();
//                 wait(1000);
//             }
//         }

//         void receivedLines(){
//             long end_time = System.currentTimeMillis();
//             long delta = end_time - start_time;
//             System.out.println("Duration: "+Long.toString(delta)+"ms.");
//             synchronized(this) {
//                 linesReceived = true;
//                 lines = l;
//                 notifyAll();
//             }
//         }
//     }
// }
