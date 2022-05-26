package persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDate;
import java.util.List;
import java.util.UUID;

@Entity
@Table(name = "offer")
@Getter
@Setter
public class OfferEntity {
	@Id
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	@OneToOne(cascade = CascadeType.ALL)
	private PolicyEntity policy;

	private LocalDate expiresOn;

	@OneToMany(mappedBy = "offer", cascade = CascadeType.ALL)
	private List<DataRowEntity> data;
}
