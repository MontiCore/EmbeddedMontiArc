package de.thesis.consumer.backend.persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

@Getter
@Setter
@Entity
@Table(name = "dataset")
public class DatasetEntity {

	@Id
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private String policy;

	@OneToMany(mappedBy = "dataset", cascade = CascadeType.ALL)
	private List<DataRowEntity> data = new ArrayList<>();
}
