function setup() {
  createCanvas(720, 480);

  // The simulation parameters.
  const gravity = createVector(0, 1); // Gravity acceleration vector.
  const boundary_friction = 0.6; // Damping of momentum on collision with boundary.
  const collison_damping = 0.7; // Damping of momentum on collision with object.
  const collision_force = 400; // Magnitude of force repeling softbody nodes on collision.
  const mouse_pull = 0.005; // Strength of mouse force.
  const damp = 0.3; // Softbody node damping.
  const stiff = 0.06; // Softbody spring stiffness.
  const mass = 1; // Softbody node mass.

  const count = 4; // Number of softbodies.
  const size = 100; // Size of softbodies.
  let resolution = 4; // Number of nodes per softbody.

  resolution = 4 * (resolution + 1)
  let softbodies = new Array(count);
  for (let i = 0; i < count; i++) {
    // Evenly space the softbodies.
    let p = i / (count - 1);
    let pos = createVector(lerp(size, width - size, p), size);
    
    /* Interpolate between green and blue,
    and then between the resultant color and white
    to create a pastel gradient of colors */
    let col = lerpColor(color('green'), color('blue'), p);
    col = lerpColor(col, color('white'), 0.6);
    
    // Initialize the softbodies with random rotations.
    let rot = random(TWO_PI);
    let body = create_square(damp, mass, stiff, resolution, size, pos, rot, col);
    softbodies[i] = body;
  }

  // Define the scene forces as functions which are applied to each softbody.
  const mouse_force = function(softbody, index) {
    // May use index to make force only apply to a subset of scene objects.
    if (mouseIsPressed) {
      let force_vector = createVector(mouseX, mouseY).sub(softbody.position);
      force_vector.mult(mouse_pull);
      softbody.external_force.add(force_vector);
    }
  }

  const gravity_force = function(softbody) {
    softbody.external_force.add(gravity);
  }

  const forces = [mouse_force, gravity_force];
  // Create the scene.
  example = new scene(softbodies, boundary_friction,
    collison_damping, collision_force, forces)
}

function draw() {
  background(255)
  textSize(20);
  textAlign(LEFT, TOP);
  fill(150, 0, 100, 255 - frameCount);
  text('Click and drag to move the squares.'.slice(0, frameCount), 20, 20)

  example.update();
}

// Function to create a square softbody.
function create_square(damping, mass, stiffness, node_count,
  size, position, rotation, color) {

  // Create empty array for the nodes.
  let nodes = new Array(node_count);
  for (let i = 0; i < nodes.length; i++) {
    let springs = [];
    for (let j = 0; j < nodes.length; j++) {
      if (i != j) {
        // Create a spring from each node to every other node.
        springs.push(new spring(0, stiffness, j));
      }
    }

    /* Create a vector based on the size of the square,
    move it up or down to generate points on a side of the square,
    and rotate it when generating a different side. */
    let normal = createVector(size / 2, 0);
    let scalar = 4 * i / nodes.length;
    let y = (scalar % 1) * size - size / 2;
    let side = HALF_PI * int(scalar % 4);
    let p = p5.Vector.add(normal, createVector(0, y)).rotate(side + rotation);
    p.add(position);

    // Create and add a node to the softbody nodes array.
    nodes[i] = new node(p, mass, damping, springs);
  }

  /* Loop over springs and alter stiffness and 
  length based on the points they're attached to. 
  
  Implementing softbody goals would be a better approach,
  but is beyond the scope of this project. */
  for (let i = 0; i < nodes.length; i++) {
    current_node = nodes[i];
    let springs = current_node.springs;
    for (let j = 0; j < springs.length; j++) {
      current_spring = springs[j];
      d = current_node.position.dist(nodes[current_spring.index].position);
      current_spring.length = d;
      current_spring.stiffness /= d;
      current_spring.stiffness *= sqrt(2) * size;
    }
  }

  return new softbody(nodes, color);
}
