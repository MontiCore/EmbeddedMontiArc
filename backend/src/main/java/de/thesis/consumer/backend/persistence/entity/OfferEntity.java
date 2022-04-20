package de.thesis.consumer.backend.persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.Id;
import java.util.UUID;

@Entity
@Getter
@Setter
public class OfferEntity {

	@Id
	@GeneratedValue
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private String policy;
}
