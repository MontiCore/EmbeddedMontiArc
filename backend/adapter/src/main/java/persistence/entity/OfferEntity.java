package persistence.entity;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import javax.persistence.*;
import java.util.List;
import java.util.UUID;

@Entity
@Table(name = "offer")
@Getter
@Setter
@AllArgsConstructor
@NoArgsConstructor
public class OfferEntity {
	@Id
	private UUID id;

	@OneToOne(cascade = CascadeType.ALL, orphanRemoval = true)
	private MetadataEntity metadata;

	@OneToMany(mappedBy = "offer", cascade = CascadeType.ALL, orphanRemoval = true)
	private List<DataRowEntity> data;
}
