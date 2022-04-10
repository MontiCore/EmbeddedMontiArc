package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Offer;

import java.util.List;

public interface OfferRepository {

	void save(Offer offer);

	List<Offer> findAll();

	Offer findBy(Long offerId);
}
