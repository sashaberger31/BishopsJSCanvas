// A basic graviational system with the Earth and a two controllable satell

// The first two coordinates are one end, the second two are the other end.

// Constants
var G = 1; // Gravitational constant
var mSat = 1; // Satellite mass
var mEarth = 1; // Earth mass
var mTarget = 1; // Target mass

// Positions
var satPos = [0,0]; // Dummy
var targetVel = [0,0]; // Dummy
var earthPos = [0,0];

// Velocities
var satVel = [0,0]; // Dummy
var targetVel = [0,0]; // Dummy
var earthVel = [0,0];

// Accelerations
var satAcc = [0,0];
var targetAcc = [0,0];
var currentBurn = [0, 0];
var netForceSat = [0, 0];

// zoomLevel > 1 ==> zoomed in, zoomLevel < 1 ==> zoomed out
var zoomLevel = 1;

class GravitationalSystem {
  // All simulated environments involving gravity should use this class
  constructor(multipoles, separations, multipoleActivations, initialCoordinates, initialVelocities, updateBooleans) {
    /*
    Purpose: initialize the variables for this gravitational system object
    Parameters:
      multipoles: 2d array of the form [[M11, M12], [M21]]
        where the Mij are the multipoles approximations of the i-th object
      separations: 3d array of the form [[[x11, y11, z11], [x12, y12, z12]], [[x21, y21, z21]]]
        where the xij, yij, and zij represent the x,y,z separations of the jth multipole
        from the ith object's center coordinate (pointing out toward the multipole).
      multipoleActivations: 2d array of the form  [[r11, r12], [r21]], where
        rij is the radius at which we begin to consider the j-th multipole of the i-th object
      initialCoordinates: 2d array of the form [[x1, y1, z1], [x2, y2, z2]]
        where the xi, yi, zi represent the x,y,z coordinates of the i-th object.
        Should have the same number of elements as multipoles
      initialVelocities: 2d array of the form [[vx1, vy1, vz1], [vx2, vy2, vz2]]
        where the vxi, vyi, vzi are the initial velocities in the x,y,z directions
        of the i-th object.
      updateBooleans: array of the form [true, false]
        where updateBooleans[i] is a boolean indicating whether or not this object
        is influenced by gravity, i.e. whether it should be updated

    Returns: none
    */
    this.multipoles = multipoles;
    this.separations = separations;
    this.multipoleActivations = multipoleActivations;
    this.coordinates = [initialCoordinates]; // First index is the step number
    this.velocities = [initialVelocities]; // First index is the step number
    this.updateBooleans = updateBooleans;
    this.G  = 1; //Something;

    /*
      TODO: Check consistency
    */
  }

  add (vector1, vector2) {
    /*
    Purpose: Add two cartesian vectors
    Parameters: Two lists containing the cartesian components
    Returns: The resultant vector
    */
    var resultant = [0,0,0];
    if (vector1.length === vector2.length) {
      for (var i = 0; i < 3; i++) {
        resultant[i] = vector1[i] + vector2[i];
      }
    } else {
      console.log("Attempted to add two unlike vectors. FAIL.");
      return [0,0,0];
    }
    return resultant;
  }

  times(scalar, vector) {
    /*
    Purpose: multiply a Cartesian vector by a scalar
    Parameters: a scalar and a vector to multiply
    Returns: the resultant vector
    */
    var resultant = [0,0,0];
    for (var i = 0; i<vector.length; i++) {
      resultant[i] = vector[i]*scalar;
    }
    return resultant;
  }

  dot (vector1, vector2) {
    /*
    Purpose: take the dot product of two cartesian vectors
    Parameters: Two lists containing the cartesian components of the vectors
    Returns: The dot product
    */
    var product = 0;
    if (vector1.length === vector2.length) {
      for (var i = 0; i < 3; i++) {
        product += vector1[i]*vector2[i];
      }
    } else {
      console.log("Attempted to add dot unlike vectors. FAIL.");
      return [0,0,0];
    }
    return product;
  }

  netAcc (externalNetForce, objNum, ourCoordinate, timestep) {
    /*
    Purpose: calculate the current net force on the object with
      number objNum (henceforth called the "test object")
    Parameters:
      externalNetForce: A vector containing any non-gravitational forces such
        as thruster burns that act on this object
      objNum: The number of the object that we want to calculate the net force of
      timestep: The currect number of timesteps we are in, i.e. the index we should
        call the array with
    Returns: Returns the net force vector on the test object
    */
    var ourMass = 0; // The mass of the test object, build it up with the multipoles
    for (var i=0; i < this.multipoles[objNum].length; i++) {
      ourMass += this.multipoles[objNum][i];
    }
    var currentNetAcc = this.times(1/ourMass, externalNetForce);

    for (var i=0; i < this.coordinates[timestep].length; i++) {
      // i is the number of the object applying force to the test object, so ignore objNum = i
      if (objNum !== i) {
        // Calculate the gravitational force object i exerts on the test object
        for (var j=0; j< this.multipoles[i].length; j++) {
          // Vector pointing from the center of our object to the center of the ith object
          var separationToCenter = this.add(this.times(-1, ourCoordinate), this.coordinates[timestep][i]);
          // Vector pointing from the center of our object to the jth multiple of the ith object
          var separationToMultipole = this.add(separationToCenter, this.times(-1, this.separations[i][j]));
          console.log("Separation of")
          console.log(objNum, i, separationToMultipole)

          // r^2 in Newton's Law of Gravitation
          var rsq = this.dot(separationToMultipole,separationToMultipole);
          // Denominator of Newton's Law of Gravitation absorbing the normalization
          var denom = rsq ** (3/2);
          console.log("Adding acc")
          console.log(objNum, this.times((this.G*this.multipoles[i][j])/denom, separationToMultipole))
          console.log("Denominator")
          console.log(denom)
          if (denom < 0.5) { // In order to prevent close-encounter unrealistic results, propel them in opposite directions
            separationToMultipole = this.times(-1, separationToMultipole)
            denom = 1
            // Warn of collision?
            alert("Collision!")
          }
          currentNetAcc = this.add(currentNetAcc, this.times((this.G*this.multipoles[i][j])/denom, separationToMultipole));
        }
      }
    }
    console.log("Net force on object:")
    console.log(objNum, timestep)
    console.log(currentNetAcc)
    return currentNetAcc;
  }

  nextStep (timestep, externalNetForce) {
    /*
    Purpose: Calculate the next iteration of the coordinates given the data at time (timestep) using the fourth-order Runge-Kutta on
      a second order ODE system which we reduced to a system of two coupled first order ODEs per object.
    Parameters:
      timestep: the step at which we are starting
    */
    var h = 1
    var newCoords = []; // Vector to hold the coordinates for the next timestep
    var newVels = []; // Vector to hold the velocities for the next timestep
    for (var i=0; i<this.coordinates[timestep].length; i++) {
      // Weights for the fourth-order RK method
      var k0 = this.times(h,this.netAcc(externalNetForce[i], i, this.coordinates[timestep][i], timestep));
      var k1 = this.times(h,this.netAcc(externalNetForce[i], i, this.add(this.coordinates[timestep][i],this.times(1/2,k0)), timestep));
      var k2 = this.times(h,this.netAcc(externalNetForce[i], i, this.add(this.coordinates[timestep][i],this.times(1/2,k1)), timestep));
      var k3 = this.times(h,this.netAcc(externalNetForce[i], i, this.add(this.coordinates[timestep][i],k2), timestep));

      var l0 = this.times(h,this.velocities[timestep][i]);
      var l1 = this.times(h,this.add(this.velocities[timestep][i],this.times(1/2,l0)));
      var l2 = this.times(h,this.add(this.velocities[timestep][i],this.times(1/2,l1)));
      var l3 = this.times(h,this.add(this.velocities[timestep][i],this.times(1/2,l0)));

      // After this mess, do the Runge-Kutta:
      var input1 = this.add(this.add(this.add(k0, this.times(2,k1)), this.times(2,k2)),k3);
      var input2 = this.add(this.add(this.add(l0, this.times(2,l1)), this.times(2,l2)),l3);
      var nextVel = this.add(this.velocities[timestep][i], this.times(1/6,input1));
      var nextCoord = this.add(this.coordinates[timestep][i], this.times(1/6,input2));
      newCoords.push(nextCoord);
      newVels.push(nextVel);
      console.log("Updating coordinates to this on object that:")
      console.log(nextCoord, i)
    }
    // Update the coordinate and velocity arrays
    this.coordinates.push(newCoords);
    this.velocities.push(newVels);
  }
}




// Count frames, track time so we can compute fps rate
var frames = 0;
var start = new Date();
var now = new Date();
console.log(start);

function calculateFPS () {
  /*
    Parameters: None
    Returns: None
    Purpose: Calculate and write to console the frame rate.
  */
  frames += 1;
  if (frames % 200 == 0) {
    now = new Date();
    msecs = now.getTime() - start.getTime();
    console.log(now.getTime());
    console.log("fps:", (frames / msecs) * 1000);
  }
}

function myKeyDown (event) {
  /*
    Parameters: event object, which contains information about the event
      that triggered the event listener.
    Returns: None, but modifies global variables which track response to event.
    Purpose: Make the animation respond to keys being pressed.
  */
  // One of the attributes of the event object is 'which,' contains the key
  //   that was pressed to trigger the event listener.
  keyCode = event.which;
  keyStr = event.key;
  console.log(event);
  console.log(keyCode);
  console.log(keyStr);

  // This will be our zoom control
  // q = zoom out, e = zoom in

  if (keyStr == "q") {
    zoomLevel -= 0.1
  }

  if (keyStr == "e") {
    zoomLevel += 0.1
  }

  if (keyStr == 'w') {
    // Move circle up
    circleVel[1] -= 1;
  }
  if (keyStr == 'a') {
    // Move circle left
    circleVel[0] -= 1;
  }
  if (keyStr == 's') {
    // Move circle down
    circleVel[1] += 1;
  }
  if (keyStr == 'd') {
    // Move circle right
    circleVel[0] += 1;
  }
}

var counterOfThings = 0;
system = new GravitationalSystem([[1000],[1]], [[[0,0,0]], [[0,0,0]]], [0], [[0,0,0], [100,0,0]], [[0,0,0], [0,3,0]], [true, true])
function drawAll()
/*
  Purpose: This is the main drawing loop.
  Inputs: None, but it is affected by what the other functions are doing
  Returns: None, but it calls itself to cycle to the next frame
*/
{

      system.nextStep(counterOfThings, [[0,0,0],[0,0,0]]);
      counterOfThings++;
      console.log("Next Timestep")
      console.log("----------------------------")



  //calculateFPS(); Uncomment LATER

  //applyVelocity(linePos, lineVel);
  //applyVelocity(circlePos, circleVel);

  // If the line hits the end of the canvas, bounce
  // Add/subtract a little speed

  /*
  if ((linePos[0] > canvas.width) || (linePos[0] < 0)) {
    lineVel[0] *= -1;
    lineVel[0] += Math.random() - 0.5;
    // console.log(lineVel);
  }
  if ((linePos[1] > canvas.height) || (linePos[1] < 0)) {
    lineVel[1] *= -1;
    lineVel[1] += Math.random() - 0.5;
    // console.log(lineVel);
  }
  if ((linePos[2] > canvas.width) || (linePos[2] < 0)) {
    lineVel[2] *= -1;
    lineVel[2] += Math.random() - 0.5;
    // console.log(lineVel);
  }
  if ((linePos[3] > canvas.height) || (linePos[3] < 0)) {
    lineVel[3] *= -1;
    lineVel[3] += Math.random() - 0.5;
    // console.log(lineVel);
  }
*/
  context.clearRect(0, 0, canvas.width, canvas.height);
  pos1x = 0.5*system.coordinates[counterOfThings][0][0] + 200
  pos1y = 0.5*system.coordinates[counterOfThings][0][1] + 200
  pos2x = 0.5*system.coordinates[counterOfThings][1][0] + 200
  pos2y = 0.5*system.coordinates[counterOfThings][1][1] + 200
  context.beginPath()
  console.log("Printing the screen")
  console.log(pos1x,pos1y,pos2x,pos2y)
  context.strokeStyle = "#000000"
  context.arc(pos1x,pos1y,10,0, 2*Math.PI);
  context.stroke();
  context.beginPath()
  context.strokeStyle = "#FF0000"
  context.arc(pos2x,pos2y,10,0, 2*Math.PI);
  context.stroke();



  setTimeout( function () {
    window.requestAnimationFrame(drawAll);
  }, 33);
}


// We don't need to log this
// Get width/height of the browser window
windowWidth = window.innerWidth;
windowHeight = window.innerHeight;
/*
console.log("Window is %d by %d", windowWidth, windowHeight);
*/
// Get the canvas, set the width and height from the window
canvas = document.getElementById("mainCanvas");
// I found that - 20 worked well for me, YMMV
canvas.width = windowWidth - 20;
canvas.height = windowHeight - 20;
canvas.style.border = "1px solid black";

// Set up the context for the animation
context = canvas.getContext("2d");


// Set up event listener for when user presses a key down.
// It then calls the function myKeyDown, passing it an event object.
document.addEventListener("keydown", myKeyDown);

// Start the circle in the center of the canvas.
circlePos = [ Math.floor(canvas.width / 2), Math.floor(canvas.height / 2), 25];

// Fire up the animation engine
window.requestAnimationFrame(drawAll);
