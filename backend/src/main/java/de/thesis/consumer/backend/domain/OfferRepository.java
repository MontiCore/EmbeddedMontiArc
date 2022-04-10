package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Offer;

import java.util.List;
import java.util.UUID;

public interface OfferRepository {

	void save(Offer offer);

	List<Offer> findAll();

	Offer findBy(UUID offerId);
}
