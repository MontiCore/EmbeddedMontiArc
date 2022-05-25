package usecases;

import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;

@AllArgsConstructor
public class GetOffersMetadataUseCase {

	private final OfferPersistencePort offerPersistencePort;

	public Iterable<Offer> getAllOffers() {
		return offerPersistencePort.findAll();
	}
}
