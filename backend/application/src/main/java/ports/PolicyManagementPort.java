package ports;

import entity.Offer;
import entity.Timer;

public interface PolicyManagementPort {

	void deployPolicy(Offer offer);

	void deployTimer(Timer timer);

	void addPxp(PolicyExecutionPoint policyExecutionPoint);
}
