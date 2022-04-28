package de.thesis.consumer.backend.persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDate;
import java.time.LocalDateTime;
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

	private LocalDateTime boughtAt;

	private LocalDate expiresOn;

	@OneToOne(cascade = CascadeType.ALL)
	@JoinColumn(name = "policy_id", referencedColumnName = "id")
	private PolicyEntity policy;

	@OneToMany(mappedBy = "dataset", cascade = CascadeType.ALL, fetch = FetchType.EAGER)
	private List<DataRowEntity> data;
}
