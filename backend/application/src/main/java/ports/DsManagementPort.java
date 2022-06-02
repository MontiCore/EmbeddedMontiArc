package ports;

import entity.Offer;

public interface DsManagementPort {
	void deployPolicy(Offer offer);
}
