package ports;

import entity.Policy;

public interface PolicyDeploymentPort {
	void deployPolicy(Policy policy);
}
