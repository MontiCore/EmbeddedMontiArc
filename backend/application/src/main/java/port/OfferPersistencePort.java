package port;

import entity.Offer;

import java.util.UUID;

public interface OfferPersistencePort {
	void save(Offer offer);

	Iterable<Offer> findAll();

	Offer findBy(UUID offerId);
}
