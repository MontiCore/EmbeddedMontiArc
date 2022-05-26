package ports;

import entity.Policy;
import entity.Timer;

public interface PolicyManagementPort {

	void deployPolicy(Policy policy);

	void deployTimer(Timer timer);
}
