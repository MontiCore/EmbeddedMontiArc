package de.thesis.consumer.backend.entities;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.Id;

@Entity
@Getter
@Setter
public class Offer {

	@Id
	@GeneratedValue
	private Long id;

	private String provider;

	private String name;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private String policy;
}
