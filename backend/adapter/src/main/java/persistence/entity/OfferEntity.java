package persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.util.List;
import java.util.UUID;

@Entity
@Table(name = "offer")
@Getter
@Setter
public class OfferEntity {
	@Id
	private UUID id;

	@OneToOne(cascade = CascadeType.ALL)
	private MetadataEntity metadata;

	@OneToMany(mappedBy = "offer", cascade = CascadeType.ALL)
	private List<DataRowEntity> data;
}
