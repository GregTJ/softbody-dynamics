/* Forward Euler spring-mass-damper softbody simulation.
Conserves rigid body momentum (linear and angular).
Could be converted to a more advanced 
Runge-Kutta solver for greater stability.
(The softbody sim in blender uses the Heun method for example...)
Damping is defined on nodes instead of springs
because it makes the code a little simpler. 
Uses an approximation of hermite radial basis function interpolation
to find the interior of the softbodies for collision. */


function scene(softbodies, boundary_friction,
  collison_damping, collision_force, forces) {
  this.softbodies = softbodies;
  this.boundary_friction = boundary_friction;
  this.collison_damping = collison_damping;
  this.collision_force = collision_force;
  this.forces = forces;

  this.update = function() {
    for (let i = 0; i < this.softbodies.length; i++) {
      current_softbody = this.softbodies[i];
      for (let j = 0; j < this.forces.length; j++) {
        current_softbody.external_force.add(this.forces[j](current_softbody, i));
      }

      // Integrate spring physics, external forces...
      current_softbody.integrate_forces();
      // Reset softbody global attributes like momentum, position, etc.
      current_softbody.reset_attributes();
      // Recalculate softbody attributes based on node attributes.
      current_softbody.calculate_attributes();
      // Calculate normals and basis function radii for collision checking.
      current_softbody.calculate_surface();
      // Apply boundary collision, damp springs, and update softbody position.
      current_softbody.damp_update_boundary(this.boundary_friction);
      // Apply softbody <-> softbody collisions.
      for (let j = 0; j < this.softbodies.length; j++) {
        if (i != j) {
          softbodies[i].collide_object(softbodies[j],
            this.collison_damping, this.collision_force);
        }
      }
      // Finally, display softbodies.
      current_softbody.display();
    }
  }
}


function softbody(nodes, color) {
  this.nodes = nodes;
  this.color = color;

  // Zero out all global softbody properties before calculating them.
  this.reset_attributes = function() {
    this.angular_momentum = createVector(0, 0, 0);
    this.linear_momentum = createVector(0, 0);
    this.position = createVector(0, 0);
    this.external_force = createVector(0, 0);
    this.mass = 0;
    this.average_distance = 0;
    this.radii = new Array(nodes.length);
    this.normals = new Array(nodes.length);
  }

  this.reset_attributes();

  this.calculate_attributes = function() {
    /* These loops basically just sum & average a bunch of properties
    of the softbody nodes to find global softbody
    momentum, position, mass, etc... */
    for (let i = 0; i < this.nodes.length; i++) {
      let current_node = this.nodes[i]
      this.linear_momentum.add(current_node.momentum);
      this.position.add(current_node.position);
      this.mass += current_node.mass;
    }
    this.linear_momentum.div(this.nodes.length)
    this.mass /= this.nodes.length;
    this.position.div(this.nodes.length);

    for (let i = 0; i < this.nodes.length; i++) {
      let current_node = this.nodes[i]
      /* This is the trickiest part,
      it calculates the angular momentum of the softbody
      by averaging the orbital momentum of each
      node around the softbody origin we calculated above. */
      this.average_distance += current_node.position.dist(this.position);
      let local_angular = p5.Vector.sub(current_node.position, this.position);
      this.angular_momentum.add(local_angular.cross(current_node.momentum));
    }
    this.angular_momentum.div(this.nodes.length)
    this.average_distance /= this.nodes.length;
  }

  this.calculate_surface = function() {
    l = this.nodes.length;
    for (let i = 0; i < l; i++) {
      /* Calculate normals between adjacent points and
      average them to find the normal at each node. */
      let prev = this.nodes[(i - 1) - l * floor((i - 1) / l)].position;
      let curr = this.nodes[i].position;
      let next = this.nodes[(i + 1) % l].position;
      let normal = p5.Vector.sub(prev, curr).rotate(HALF_PI).normalize();
      normal.add(p5.Vector.sub(curr, next).rotate(HALF_PI).normalize())
      this.normals[i] = normal.normalize();

      /* Simplisitic calculation of basis function radii, 
      sufficient for roughly convex shapes whose points 
      are roughly equidistant to the origin. */
      this.radii[i] = curr.dist(this.position);
    }
  }

  this.volume_field = function(point, reg = 1) {
    /* Citation: http://www.3dgp.net/paper/2016/A%20Closed-Form%20Formulation%20of%20HRBF-Based%20Surface%20Reconstruction.pdf (pg. 5, eqn. 10)
    
    This function returns a negative number if the point is inside the shape,
    positive if outside, 0 if exactly on the boundary.
    
    This simulation leverages the properties of this function to 
    generate a repulsive force roughly proportional to the depth at which a
    node intersects a softbody.
    
    A simpler collision scheme may use the crossing number
    algorithm (point in polygon), used in the game Asteroids for example. 
    
    It is also possible to find the closed form of the gradient of this function
    for more sophisticated collision schemes. */

    var hrbf = 0;
    for (let i = 0; i < this.nodes.length; i++) {
      let position = this.nodes[i].position;
      let radius = this.radii[i];
      let normal = this.normals[i];
      let d = point.dist(position);
      let d_scl = d / radius
      if (0 <= d_scl && d_scl <= 1) {
        let r_sqr = pow(radius, 2)
        let v1 = p5.Vector.mult(normal, r_sqr / (20 + reg * r_sqr));
        let v2 = p5.Vector.sub(point, position);
        v2.mult(20 * pow(d - radius, 3) / pow(radius, 5));
        hrbf -= v1.dot(v2);
      }
    }
    return hrbf;
  }

  this.display = function(normals = false) {
    fill(this.color);
    noStroke();
    curveTightness(0.25);

    // Draw a filled curve around the shape.
    beginShape();
    for (let i = 0; i < this.nodes.length; i++) {
      let index = (i + 1) % this.nodes.length
      let position = this.nodes[index].position
      curveVertex(position.x, position.y);
    }
    endShape(CLOSE);

    // Draw the node normals as scaled by the node radii.
    if (normals) {
      stroke(0);
      strokeWeight(1);
      for (let i = 0; i < this.nodes.length; i++) {
        let position = this.nodes[i].position;
        let radius = this.radii[i];
        let normal = p5.Vector.mult(this.normals[i], radius / 2);
        normal.add(position);
        noFill();
        line(position.x, position.y, normal.x, normal.y);
      }
    }
  }

  // See scene and node constructors for details on what these do.
  this.integrate_forces = function() {
    for (let i = 0; i < this.nodes.length; i++) {
      current_node = this.nodes[i];
      current_node.integrate_external(this.external_force);
      current_node.integrate_springs(this.nodes);
    }
  }

  this.damp_update_boundary = function(friction) {
    for (let i = 0; i < this.nodes.length; i++) {
      current_node = this.nodes[i];
      current_node.calculate_global(this.angular_momentum, this.linear_momentum,
        this.position, this.average_distance)
      current_node.apply_damping();
      current_node.update();
      current_node.boundary_collide(friction);
    }
  }

  this.collide_object = function(object, damping, force) {
    for (let i = 0; i < this.nodes.length; i++) {
      this.nodes[i].collide_object(object, this.normals[i], damping, force);
    }
  }
}

function node(position, mass, damping, springs) {
  this.position = position;
  this.mass = mass;
  this.damping = damping;
  this.springs = springs;
  this.momentum = createVector(0, 0);
  this.global_momentum = createVector(0, 0);

  this.integrate_springs = function(nodes) {
    // For every spring of this node;
    for (let i = 0; i < this.springs.length; i++) {
      let goal_spring = this.springs[i];
      let goal_node = nodes[goal_spring.index];

      /* Calculate goal position between the two masses
         by finding the unit normal vector between the masses,
         scaling it by the spring length,
         then adding the location of the current node. 
         
         By Hooke's law, the force exerted by the spring should
         be directly propotional to the distance between the
         mass and the rest point of the spring.
         
         If you alter this code slightly, 
         changing the falloff law to the inverse square law,
         this becomes a newtonian gravity sim. */
      let goal_position = p5.Vector.sub(this.position, goal_node.position);
      goal_position.normalize();
      goal_position.mult(goal_spring.length);
      goal_position.add(goal_node.position);

      /* Find acceleration vector (goal_vector) by subtracting
         the current node's position from the goal position. */
      let goal_vector = p5.Vector.sub(goal_position, this.position);

      // Scale the acceleration vector by the stiffness coefficient.
      goal_vector.mult(goal_spring.stiffness);
      
      // Second half of Newton's second law of motion.
      goal_vector.div(this.mass);

      // Integrate acceleration into momentum.
      this.momentum.add(goal_vector);
    }
  }
  
  this.calculate_global = function(softbody_angular, softbody_linear,
    softbody_position, average_distance) {
     /* The vector math here recovers the average
    orbital momentum from the softbody angular momentum. */
    let local_position = p5.Vector.sub(this.position, softbody_position);
    let local_angular = local_position.copy().rotate(radians(softbody_angular.z));
    local_angular.sub(local_position);
    local_angular.div(average_distance);
    this.global_momentum = p5.Vector.add(softbody_linear, local_angular);
  }

  this.apply_damping = function() {
    // Apply spring damping to node momentum, conserving softbody momentum.
    this.momentum.sub(this.global_momentum);
    this.momentum.mult(1 - this.damping);
    this.momentum.add(this.global_momentum);
  }

  this.integrate_external = function(softbody_external) {
    // Simply add the external forces to the node momentum.
    this.momentum.add(softbody_external);
  }

  this.update = function() {
    // Integrate momentum into position.
    this.position.add(this.momentum);
  }

  this.boundary_collide = function(friction) {
    // Check if node is outside bounds of simulation.
    if (this.position.x < 0 || this.position.x > width) {
      // Flip momentum in appropriate direction if it is.
      this.momentum.x *= -1
      // Apply boundary friction to momentum.
      this.momentum.mult(1 - friction);
    }

    // Repeat for horizontal walls of boundary.
    if (this.position.y < 0 || this.position.y > height) {
      this.momentum.y *= -1
      this.momentum.mult(1 - friction);
    }

    /* In addition to flipping momentum and applying friction,
    constrain position to within bounds of simulation to prevent clipping errors. */
    this.position.x = constrain(this.position.x, 0, width);
    this.position.y = constrain(this.position.y, 0, height);
  }

  // Very simplistic and inacurrate but visually convincing collision.
  this.collide_object = function(object, normal, damping, force) {
    scalar = object.volume_field(this.position);
    // Check if node is inside the object.
    if (scalar < 0) {
      this.momentum.sub(this.global_momentum);
      /* Push node away from softbody, in the direction of the nodes normal,
      as scaled by the distance from the surface of the softbody. */
      this.momentum.sub(p5.Vector.mult(normal, force * -scalar));
      this.momentum.mult(1 - damping);
      this.momentum.add(this.global_momentum);
    }
  }
}


// Helper constructor for spring attributes.
function spring(length, stiffness, index) {
  this.length = length;
  this.stiffness = stiffness;
  this.index = index;
}
