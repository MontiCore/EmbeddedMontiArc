
public class FlexRayOperationMode {
	
	private static final double BIT_ERROR_RATE_REDUNDANCY = 0.001;
	
	private static final double BIT_ERROR_RATE_MAX_DATA = 0.01;
	
	//in MBit/s
	private static final int DATA_RATE_REDUNDANCY = 10;
	
	//in MBit/s
	private static final int DATA_RATE_MAX_DATA = 20;
		
	private FlexRayOperationModeEnum mode;
	
	public FlexRayOperationMode(FlexRayOperationModeEnum mode) {
		this.mode = mode;
	}

	public int getDataRate() {
		if(this.mode == FlexRayOperationModeEnum.REDUNDANCY) {
			return DATA_RATE_REDUNDANCY;
		}
		else if(this.mode == FlexRayOperationModeEnum.MAX_DATA) {
			return DATA_RATE_MAX_DATA;
		}
		return -1;
	}

	public double getErrorRate() {
		if(this.mode == FlexRayOperationModeEnum.REDUNDANCY) {
			return BIT_ERROR_RATE_REDUNDANCY;
		}
		else if(this.mode == FlexRayOperationModeEnum.MAX_DATA) {
			return BIT_ERROR_RATE_MAX_DATA;
		}
		return -1;
	}

	
	
}
