package de.thesis.consumer.backend.persistence.entity;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Table;

@Entity
@Table(name = "policy")
@Getter
@Setter
@AllArgsConstructor
@NoArgsConstructor
public class PolicyEntity {
	@Id
	private String id;
	@Column(length = 2048)
	private String rawValue;
}
