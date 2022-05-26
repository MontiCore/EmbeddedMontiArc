package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDate;

@Entity
@Table(name = "metadata")
@Getter
@Setter
public class MetadataEntity {
	@Id
	@GeneratedValue
	private Long id;

	String title;

	String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private LocalDate expiresOn;

	@OneToOne(cascade = CascadeType.ALL)
	private PolicyEntity policy;
}

