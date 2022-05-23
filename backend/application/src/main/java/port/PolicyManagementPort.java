package port;

import entity.Policy;
import entity.Timer;

public interface PolicyManagementPort {

	boolean isValid(Policy policy);
	void deployPolicy(Policy policy);

	void deployTimer(Timer timer);
}
