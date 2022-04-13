package de.thesis.consumer.backend.entities;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

@Entity
@Getter
@Setter
public class Dataset {

	@Id
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private String policy;

	@OneToMany(mappedBy = "dataset", cascade = CascadeType.ALL)
	private List<TruckData> truckData = new ArrayList<>();
}
