package de.thesis.provider.backend.integration;

import java.util.UUID;


public class IntegrationTestUtils {
	public static boolean isInitialOffer(UUID offerId) {
		return offerId.toString().equals("6355f4d9-6361-4a55-89d8-57bcd5666532") ||
				offerId.toString().equals("d9c48beb-24a6-4663-8e6e-a361ca74a114") ||
				offerId.toString().equals("f10d7f03-ae67-4c51-ba92-ab62f7962973");
	}
}
