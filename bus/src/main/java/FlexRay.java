import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class FlexRay implements Bus {
	
	//in byte
	private static final int HEADER_SIZE = 5;
	
	//in byte
	private static final int TRAILER_SIZE = 3;
	
	//in byte
	private static final int MAX_PAYLOAD_LEN = 254;
	
	//dynamic?
	private static final int STATIC_SLOTS = 1;
	
	//dynamic?
	private static final int DYNAMIC_SLOTS = 4;
	
	private int connectedComponents;
	
	private List<BusMessage> messages = new ArrayList<BusMessage>();

	FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);
	
	public FlexRay(int connectedComponents) {
		this.connectedComponents = connectedComponents;
	}
	
	//in mikro sekunden
	private int getSlotSize() {
		return (int)Math.ceil(((HEADER_SIZE + TRAILER_SIZE + MAX_PAYLOAD_LEN) * 8)/(double)mode.getDataRate());
	}
	
	//in mikro sekunden
	private int getCycleTime() {
		//guard space?
		return this.getSlotSize() * (this.connectedComponents + DYNAMIC_SLOTS);
	}
	
	private Map<Integer, List<BusMessage>> calculateStartCycles(){
		Map<Integer, List<BusMessage>> messagesByStartCycle = new HashMap<Integer, List<BusMessage>>();
		for(BusMessage message : messages) {
			int startCycle = message.getRequestTime()/getCycleTime();
			List<BusMessage> startCycleMessages = messagesByStartCycle.getOrDefault(startCycle, new ArrayList<BusMessage>());
			startCycleMessages.add(message);
			messagesByStartCycle.put(startCycle, startCycleMessages);
		}
		return messagesByStartCycle;
	}
	
	private int calculateFinishTime() {
		//wasted mini-slots?
		Map<Integer, List<BusMessage>> messagesByStartCycle = calculateStartCycles();
		
		HashMap<Integer, List<BusMessage>> messagesByControllerId = new HashMap<Integer, List<BusMessage>>();
		for(int cylce = 0; !messagesByStartCycle.isEmpty(); cylce++) {
			List<BusMessage> cycleMessages = messagesByStartCycle.remove(cylce);
			if(cycleMessages != null) {
				for(BusMessage message : cycleMessages) {
					List<BusMessage> controllerMessages = messagesByControllerId.getOrDefault(message.getControllerID(), new ArrayList<BusMessage>());
					controllerMessages.add(message);
					messagesByControllerId.put(message.getControllerID(), controllerMessages);
				}
			}
			//static segment
			for(List<BusMessage> controllerMessages : messagesByControllerId.values()) {
				int transmitted = 0;
				while(transmitted < MAX_PAYLOAD_LEN) {
					controllerMessages.sort(new BusMessageComparator());
					BusMessage message = controllerMessages.get(0);
					if(message != null) {
						transmitted += message.transmitBytes(MAX_PAYLOAD_LEN - transmitted);
						if(transmitted >= 0) {
							if(message.isTransmitted()) {
								controllerMessages.remove(message);
							}
						}
						//error
						else {
						}
					}
				}
			}
			List<BusMessage> allMessages = messagesByControllerId.values().stream()
			        .flatMap(List::stream)
			        .collect(Collectors.toList());
			//dynamic segment
			int transmitted = 0;
			while(transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
				allMessages.sort(new BusMessageComparator());
				BusMessage message = allMessages.get(0);
				if(message != null) {
					transmitted += message.transmitBytes(MAX_PAYLOAD_LEN - transmitted);
					if(transmitted >= 0) {
						if(message.isTransmitted()) {
							allMessages.remove(message);
							List<BusMessage> controllerMessages = messagesByControllerId.remove(message.getControllerID());
							controllerMessages.remove(message);
							if(!controllerMessages.isEmpty()) {
								messagesByControllerId.put(message.getControllerID(), controllerMessages);
							}
							
						}
					}
					//error
					else {
					}
				}
			}
		}
		while(!messagesByControllerId.isEmpty()) {
			//static segment
			for(List<BusMessage> controllerMessages : messagesByControllerId.values()) {
				int transmitted = 0;
				while(transmitted < MAX_PAYLOAD_LEN) {
					controllerMessages.sort(new BusMessageComparator());
					BusMessage message = controllerMessages.get(0);
					if(message != null) {
						transmitted += message.transmitBytes(MAX_PAYLOAD_LEN - transmitted);
						if(transmitted >= 0) {
							if(message.isTransmitted()) {
								controllerMessages.remove(message);
							}
						}
						//error
						else {
						}
					}
				}
			}
			List<BusMessage> allMessages = messagesByControllerId.values().stream()
			        .flatMap(List::stream)
			        .collect(Collectors.toList());
			//dynamic segment
			int transmitted = 0;
			while(transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
				allMessages.sort(new BusMessageComparator());
				BusMessage message = allMessages.get(0);
				if(message != null) {
					transmitted += message.transmitBytes(MAX_PAYLOAD_LEN - transmitted);
					if(transmitted >= 0) {
						if(message.isTransmitted()) {
							allMessages.remove(message);
							List<BusMessage> controllerMessages = messagesByControllerId.remove(message.getControllerID());
							controllerMessages.remove(message);
							if(!controllerMessages.isEmpty()) {
								messagesByControllerId.put(message.getControllerID(), controllerMessages);
							}
							
						}
					}
					//error
					else {
					}
				}
			}
		}
		return -1;
	}
	
	
	@Override
	public int setData(String key, BusMessage msg) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public BusMessage getData(String key) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Map<String, BusMessage> getAllData() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String[] getImportNames() {
		// TODO Auto-generated method stub
		return null;
	}

}
