package ports;

import entity.Offer;

import java.util.UUID;

public interface OfferPersistencePort {
	void save(Offer offer);

	Offer findById(UUID id);

	Iterable<Offer> findAll();

	Iterable<Offer> findAllBought();
}
