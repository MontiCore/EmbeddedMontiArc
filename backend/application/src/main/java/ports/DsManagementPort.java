package ports;

import entity.Policy;

public interface DsManagementPort {
	void deployPolicy(Policy policy);
}
