/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons;

import org.apache.commons.math3.linear.RealVector;

/**
 * this class provides all information for a vertex data structur for a general graph.
 * It is widely used in this function block setting.
 *
 * Created by Christoph Grüne on 22.12.2016.
 * @author Christoph Grüne.
 */
public class Vertex {

    private Long id;
    private Long osmId;
    private Double maximumSteeringAngle;
    private RealVector position;
    private boolean intersectionNode = false;

    public Vertex(Long id, Long osmID, RealVector position, Double maximumSteeringAngle) {
        this.id = id;
        this.osmId = osmID;
        this.position = position;
        this.maximumSteeringAngle = maximumSteeringAngle;
    }

    /**
     * copy constructor for Vertex
     */
    public Vertex copy() {
        return new Vertex(this.id.longValue(), this.osmId.longValue(), this.position.copy(), this.maximumSteeringAngle.doubleValue());
    }

    /**
     * Setter for maximumVelocity
     *
     * @param maximumSteeringAngle the new maximumVelocity
     */
    public void setMaximumSteeringAngle(double maximumSteeringAngle) {
        this.maximumSteeringAngle = maximumSteeringAngle;
    }

    /**
     * Function that sets intersectionNode
     *
     * @param intersectionNode New value for intersectionNode
     */
    public void setIntersectionNode(boolean intersectionNode) {
        this.intersectionNode = intersectionNode;
    }

    /**
     * Function that returns intersectionNode
     *
     * @return Value for intersectionNode
     */
    public boolean isIntersectionNode() {
        return intersectionNode;
    }

    /**
     * Getter for the ID of this vertex
     *
     * @return the ID
     */
    public Long getId() {
        return new Long(id.longValue());
    }
    
    /**
     * Setter for vertex ID
     * 
     * @param id
     */
    public void setId(long id) {
		this.id = id;
	}

    /**
     * Getter for the OSM ID of this vertex
     *
     * @return the OSM ID
     */
    public Long getOsmId() {
        return new Long(osmId.longValue());
    }

    /**
     * Getter for the maximal velocity one can drive at this vertex
     *
     * @return the maximal velocity
     */
    public Double getMaximumSteeringAngle() {
        return new Double(maximumSteeringAngle.doubleValue());
    }

    /**
     * Getter for the position of this vertex
     *
     * @return the position
     */
    public RealVector getPosition() {
        return position.copy();
    }

    /**
     * Test for the equality of two vertices. If all attributes are the same the vertices are considered to be equal.
     *
     * @param obj the object to test
     * @return if the objects are equal
     */
    @Override
    public boolean equals(Object obj) {
        if(obj.getClass() != this.getClass()) {
            return false;
        }

        Vertex otherVertex = (Vertex) obj;
        if(!otherVertex.getMaximumSteeringAngle().equals(this.getMaximumSteeringAngle())) {
            return false;
        } else if(!otherVertex.getOsmId().equals(this.getOsmId())) {
            return false;
        } else if(!otherVertex.getId().equals(this.getId())) {
            return false;
        } else if (!otherVertex.getPosition().equals(this.getPosition())) {
            return false;
        }
        return true;
    }
}
