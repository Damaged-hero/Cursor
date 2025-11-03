// Input state: keyboard keys and mouse buttons/position
var Input = {
 keys: [],
 mouse: {
 left: false,
 right: false,
 middle: false,
 x:0,
 y:0
 }
};
// Reserve a reasonable keycode range and initialize to false
for (var i =0; i <230; i++) {
 Input.keys.push(false);
}

// Keyboard events: store key down/up state by keyCode
document.addEventListener("keydown", function(event) {
 Input.keys[event.keyCode] = true;
});
document.addEventListener("keyup", function(event) {
 Input.keys[event.keyCode] = false;
});

// Mouse button events: update left/middle/right booleans
// Note: use strict equality to read event.button (0=left,1=middle,2=right)
document.addEventListener("mousedown", function(event) {
 if (event.button ===0) {
 Input.mouse.left = true;
 }
 if (event.button ===1) {
 Input.mouse.middle = true;
 }
 if (event.button ===2) {
 Input.mouse.right = true;
 }
});
document.addEventListener("mouseup", function(event) {
 if (event.button ===0) {
 Input.mouse.left = false;
 }
 if (event.button ===1) {
 Input.mouse.middle = false;
 }
 if (event.button ===2) {
 Input.mouse.right = false;
 }
});

// Track mouse position relative to the client area
document.addEventListener("mousemove", function(event) {
 Input.mouse.x = event.clientX;
 Input.mouse.y = event.clientY;
});

// Canvas setup: full-window2D drawing surface
var canvas = document.createElement("canvas");
document.body.appendChild(canvas);
// width/height set to the window size
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;
canvas.style.position = "absolute";
canvas.style.left = "0px";
canvas.style.top = "0px";
// Prevent page scroll so the canvas stays fixed
document.body.style.overflow = "hidden";
var ctx = canvas.getContext("2d");

// Resize handling
window.addEventListener('resize', function() {
 canvas.width = window.innerWidth;
 canvas.height = window.innerHeight;
});

// A target provider gives the (x,y) the creature should follow; setups set this as needed.
var targetProvider = function() { return { x: Input.mouse.x, y: Input.mouse.y }; };

// Segment/limb system classes ------------------------------------------------

// Global segment counter (useful for debugging / profiling)
var segmentCount =0;

/**
 * Segment - represents a single link/joint in a kinematic chain.
 * Fields:
 * - parent: parent node (can be a Creature or another Segment)
 * - children: array of connected child segments
 * - size: fixed distance from parent to this joint
 * - relAngle: angle relative to parent
 * - defAngle: default (rest) relative angle
 * - absAngle: absolute angle (relative to x-axis)
 * - range: allowed angular range around defAngle
 * - stiffness: how strongly it resists deviation from defAngle
 * - x,y: world coordinates of this joint end
 */
class Segment {
 constructor(parent, size, angle, range, stiffness) {
 segmentCount++;
 this.isSegment = true;
 this.parent = parent;
 // Attach to parent's children array when present
 if (typeof parent.children == "object") {
 parent.children.push(this);
 }
 this.children = [];
 this.size = size;
 this.relAngle = angle;
 this.defAngle = angle;
 this.absAngle = parent.absAngle + angle;
 this.range = range;
 this.stiffness = stiffness;
 // Compute initial world position and normalize angles
 this.updateRelative(false, true);
 }

 /**
 * updateRelative(iter, flex)
 * - Normalizes relAngle to be close to defAngle (wraps within -PI..PI)
 * - Applies optional 'flex' clamping/smoothing toward defAngle using stiffness
 * - Computes absolute angle and world (x,y) position based on parent
 * - If iter=true, recurses into children
 */
 updateRelative(iter, flex) {
 // Normalize relAngle so it's within +/-PI of defAngle
 this.relAngle =
 this.relAngle -
2 *
 Math.PI *
 Math.floor((this.relAngle - this.defAngle) /2 / Math.PI +1 /2);

 if (flex) {
 // Apply stiffness-limited pull toward defAngle, and clamp to [defAngle-range/2, defAngle+range/2]
 this.relAngle = Math.min(
 this.defAngle + this.range /2,
 Math.max(
 this.defAngle - this.range /2,
 (this.relAngle - this.defAngle) / this.stiffness + this.defAngle
 )
 );
 }

 this.absAngle = this.parent.absAngle + this.relAngle;
 // Compute world position of this segment's end
 this.x = this.parent.x + Math.cos(this.absAngle) * this.size;
 this.y = this.parent.y + Math.sin(this.absAngle) * this.size;

 if (iter) {
 for (var i =0; i < this.children.length; i++) {
 this.children[i].updateRelative(iter, flex);
 }
 }
 }

 /**
 * draw(iter)
 * - Adds a line from parent to this segment's end to the current path
 * - If iter=true, adds child segments recursively
 */
 draw(iter) {
 ctx.moveTo(this.parent.x, this.parent.y);
 ctx.lineTo(this.x, this.y);
 if (iter) {
 for (var i =0; i < this.children.length; i++) {
 this.children[i].draw(true);
 }
 }
 }

 /**
 * follow(iter)
 * - Pulls this segment toward a configuration where its end is 'size' units
 * from its parent, preserving chain length. Useful for inverse-kinematics.
 * - Recomputes angles and updates children when iter=true.
 */
 follow(iter) {
 var x = this.parent.x;
 var y = this.parent.y;
 var dist = ((this.x - x) **2 + (this.y - y) **2) **0.5;
 // Reposition this segment's end to be exactly 'size' away from parent along current direction
 this.x = x + this.size * (this.x - x) / dist;
 this.y = y + this.size * (this.y - y) / dist;
 this.absAngle = Math.atan2(this.y - y, this.x - x);
 this.relAngle = this.absAngle - this.parent.absAngle;
 // Apply constraints and propagate to children
 this.updateRelative(false, true);

 if (iter) {
 for (var i =0; i < this.children.length; i++) {
 this.children[i].follow(true);
 }
 }
 }
}

/**
 * LimbSystem - basic inverse-kinematic limb control.
 * - end: the last Segment in the chain (the foot/hand)
 * - length: requested number of segments taken from end towards the root
 * - speed: how far the limb pulls each update (affects step distance)
 * - creature: owning Creature, added to creature.systems
 */
class LimbSystem {
 constructor(end, length, speed, creature) {
 this.end = end;
 this.length = Math.max(1, length);
 this.creature = creature;
 this.speed = speed;
 creature.systems.push(this);
 this.nodes = [];

 var node = end;
 // Gather 'length' nodes from the end back toward the root
 for (var i =0; i < length; i++) {
 this.nodes.unshift(node);
 node = node.parent;
 if (!node.isSegment) {
 // Reached the root (non-segment), adjust actual length
 this.length = i +1;
 break;
 }
 }
 // The hip is the parent of the first node in nodes[]
 this.hip = this.nodes[0].parent;
 }

 /**
 * moveTo(x,y)
 * - Repositions the limb so the end tries to reach (x,y).
 * - Uses an iterative backward-forward pass: positions nodes from end toward hip.
 */
 moveTo(x, y) {
 // Ensure local angles are up-to-date before solving
 this.nodes[0].updateRelative(true, true);

 var dist = ((x - this.end.x) **2 + (y - this.end.y) **2) **0.5;
 var len = Math.max(0, dist - this.speed);

 // Place nodes from tip backwards, keeping each node at its size distance from the next
 for (var i = this.nodes.length -1; i >=0; i--) {
 var node = this.nodes[i];
 var ang = Math.atan2(node.y - y, node.x - x);
 node.x = x + len * Math.cos(ang);
 node.y = y + len * Math.sin(ang);
 x = node.x;
 y = node.y;
 len = node.size;
 }

 // Recompute angles and update any child branches not in this.nodes[]
 for (var i =0; i < this.nodes.length; i++) {
 var node = this.nodes[i];
 node.absAngle = Math.atan2(node.y - node.parent.y, node.x - node.parent.x);
 node.relAngle = node.absAngle - node.parent.absAngle;
 for (var ii =0; ii < node.children.length; ii++) {
 var childNode = node.children[ii];
 if (!this.nodes.includes(childNode)) {
 childNode.updateRelative(true, false);
 }
 }
 }
 }

 update() {
 // Default behavior: pull the limb toward the current mouse position
 this.moveTo(Input.mouse.x, Input.mouse.y);
 }
}

/**
 * LegSystem - specialized LimbSystem that chooses foot placement goals
 * and steps forward/back according to a basic gait rule.
 */
class LegSystem extends LimbSystem {
 constructor(end, length, speed, creature) {
 super(end, length, speed, creature);
 this.goalX = end.x;
 this.goalY = end.y;
 // step states:0 = stand still,1 = move forward,2 = move toward foothold
 this.step =0;
 this.forwardness =0;

 // Precompute typical reach for placing the foot relative to hip
 this.reach =
0.9 * ((this.end.x - this.hip.x) **2 + (this.end.y - this.hip.y) **2) **0.5;

 // Swing behavior determines direction of foot placement relative to creature heading
 var relAngle =
 this.creature.absAngle -
 Math.atan2(this.end.y - this.hip.y, this.end.x - this.hip.x);
 relAngle -=2 * Math.PI * Math.floor(relAngle /2 / Math.PI +1 /2);
 this.swing = -relAngle + (2 * (relAngle <0) -1) * Math.PI /2;
 this.swingOffset = this.creature.absAngle - this.hip.absAngle;
 }

 update(x, y) {
 // Try to move current foot toward its goal position
 this.moveTo(this.goalX, this.goalY);

 if (this.step ==0) {
 // If the current end has drifted far from the goal, pick a new foothold
 var dist =
 ((this.end.x - this.goalX) **2 + (this.end.y - this.goalY) **2) **0.5;
 if (dist >1) {
 this.step =1;
 // Choose new random-ish goal in front/back depending on swing and hip
 this.goalX =
 this.hip.x +
 this.reach *
 Math.cos(this.swing + this.hip.absAngle + this.swingOffset) +
 (2 * Math.random() -1) * this.reach /2;
 this.goalY =
 this.hip.y +
 this.reach *
 Math.sin(this.swing + this.hip.absAngle + this.swingOffset) +
 (2 * Math.random() -1) * this.reach /2;
 }
 } else if (this.step ==1) {
 // While stepping, evaluate forwardness; stop when forwardness stabilizes
 var theta =
 Math.atan2(this.end.y - this.hip.y, this.end.x - this.hip.x) -
 this.hip.absAngle;
 var dist =
 ((this.end.x - this.hip.x) **2 + (this.end.y - this.hip.y) **2) **0.5;
 var forwardness2 = dist * Math.cos(theta);
 var dF = this.forwardness - forwardness2;
 this.forwardness = forwardness2;
 if (dF * dF <1) {
 this.step =0;
 this.goalX = this.hip.x + (this.end.x - this.hip.x);
 this.goalY = this.hip.y + (this.end.y - this.hip.y);
 }
 }
 }
}

/**
 * Creature - root object with position, heading and movement parameters.
 * Children of a Creature are typically Segments that form its body.
 */
class Creature {
 constructor(
 x,
 y,
 angle,
 fAccel,
 fFric,
 fRes,
 fThresh,
 rAccel,
 rFric,
 rRes,
 rThresh
 ) {
 this.x = x;
 this.y = y;
 this.absAngle = angle;

 // Forward motion state and parameters
 this.fSpeed =0;
 this.fAccel = fAccel;
 this.fFric = fFric;
 this.fRes = fRes;
 this.fThresh = fThresh; // minimal distance to keep moving

 // Rotational motion state and parameters
 this.rSpeed =0;
 this.rAccel = rAccel;
 this.rFric = rFric;
 this.rRes = rRes;
 this.rThresh = rThresh; // max heading difference before rotation

 // Hierarchy: children segments and attached limb systems
 this.children = [];
 this.systems = [];
 }

 /**
 * follow(x,y)
 * - High-level controller: move creature toward target (x,y), rotate toward it,
 * update child segments and limb systems, then draw.
 */
 follow(x, y) {
 var dist = ((this.x - x) **2 + (this.y - y) **2) **0.5;
 var angle = Math.atan2(y - this.y, x - this.x);

 // Update forward acceleration: systems that are stepping reduce effective accel
 var accel = this.fAccel;
 if (this.systems.length >0) {
 var sum =0;
 for (var i =0; i < this.systems.length; i++) {
 sum += this.systems[i].step ==0;
 }
 accel *= sum / this.systems.length;
 }
 this.fSpeed += accel * (dist > this.fThresh);
 this.fSpeed *=1 - this.fRes;
 this.speed = Math.max(0, this.fSpeed - this.fFric);

 // Update rotation: turn toward target if angle difference is significant
 var dif = this.absAngle - angle;
 dif -=2 * Math.PI * Math.floor(dif / (2 * Math.PI) +1 /2);
 if (Math.abs(dif) > this.rThresh && dist > this.fThresh) {
 this.rSpeed -= this.rAccel * (2 * (dif >0) -1);
 }
 this.rSpeed *=1 - this.rRes;
 if (Math.abs(this.rSpeed) > this.rFric) {
 this.rSpeed -= this.rFric * (2 * (this.rSpeed >0) -1);
 } else {
 this.rSpeed =0;
 }

 // Apply motion
 this.absAngle += this.rSpeed;
 this.absAngle -=2 * Math.PI * Math.floor(this.absAngle / (2 * Math.PI) +1 /2);
 this.x += this.speed * Math.cos(this.absAngle);
 this.y += this.speed * Math.sin(this.absAngle);

 // Temporarily add PI so children treat the 'front' consistently (existing code logic)
 this.absAngle += Math.PI;
 for (var i =0; i < this.children.length; i++) {
 this.children[i].follow(true, true);
 }
 for (var i =0; i < this.systems.length; i++) {
 this.systems[i].update(x, y);
 }
 this.absAngle -= Math.PI;

 // Draw creature base and recursive children
 this.draw(true);
 }

 /**
 * draw(iter)
 * - Adds a simple triangular marker for creature heading to the current path
 * - If iter=true adds child segments recursively
 */
 draw(iter) {
 var r =4;
 ctx.arc(
 this.x,
 this.y,
 r,
 Math.PI /4 + this.absAngle,
7 * Math.PI /4 + this.absAngle
 );
 ctx.moveTo(
 this.x + r * Math.cos(7 * Math.PI /4 + this.absAngle),
 this.y + r * Math.sin(7 * Math.PI /4 + this.absAngle)
 );
 ctx.lineTo(
 this.x + r * Math.cos(this.absAngle) *2 **0.5,
 this.y + r * Math.sin(this.absAngle) *2 **0.5
 );
 ctx.lineTo(
 this.x + r * Math.cos(Math.PI /4 + this.absAngle),
 this.y + r * Math.sin(Math.PI /4 + this.absAngle)
 );
 if (iter) {
 for (var i =0; i < this.children.length; i++) {
 this.children[i].draw(true);
 }
 }
 }
}

// Initialization helpers -----------------------------------------------------

var critter;

/**
 * setupSimple
 * - Creates a single long chain of segments attached to a Creature.
 * - The creature follows the mouse.
 */
function setupSimple() {
 // (x,y,angle,fAccel,fFric,fRes,fThresh,rAccel,rFric,rRes,rThresh)
 var localCritter = new Creature(
 window.innerWidth /2,
 window.innerHeight /2,
0,
12,
1,
0.5,
16,
0.5,
0.085,
0.5,
0.3
 );
 var node = localCritter;
 // Build a long chain of small segments
 for (var i =0; i <128; i++) {
 var node = new Segment(node,8,0,3.14159 /2,1);
 }
 critter = localCritter;
 // follow mouse
 targetProvider = function() { return { x: Input.mouse.x, y: Input.mouse.y }; };
}

/**
 * setupTentacle
 * - Builds a tentacle-like chain with a LimbSystem that reaches toward center.
 */
function setupTentacle() {
 critter = new Creature(
 window.innerWidth /2,
 window.innerHeight /2,
0,
12,
1,
0.5,
16,
0.5,
0.085,
0.5,
0.3
 );
 var node = critter;
 for (var i =0; i <32; i++) {
 var node = new Segment(node,8,0,2,1);
 }
 var tentacle = new LimbSystem(node,32,8, critter);
 // follow center
 targetProvider = function() { return { x: canvas.width /2, y: canvas.height /2 }; };
}

/**
 * setupArm
 * - Small articulated arm example. (Note: original code's LimbSystem call omits speed)
 */
function setupArm() {
 var localCritter = new Creature(
 window.innerWidth /2,
 window.innerHeight /2,
0,
12,
1,
0.5,
16,
0.5,
0.085,
0.5,
0.3
 );
 var node = localCritter;
 // Create3 long segments for an arm
 for (var i =0; i <3; i++) {
 var node = new Segment(node,80,0,3.1416,1);
 }
 // Fix: provide a sensible speed parameter and creature
 var tentacle = new LimbSystem(node,3,8, localCritter);
 critter = localCritter;
 // follow center
 targetProvider = function() { return { x: canvas.width /2, y: canvas.height /2 }; };
}

/**
 * setupTestSquid
 * - Builds a multi-legged 'squid' with many joints and LegSystem controllers.
 */
function setupTestSquid(size, legs) {
 critter = new Creature(
 window.innerWidth /2,
 window.innerHeight /2,
0,
 size *10,
 size *3,
0.5,
16,
0.5,
0.085,
0.5,
0.3
 );
 var legNum = legs;
 var jointNum =32;
 for (var i =0; i < legNum; i++) {
 var node = critter;
 var ang = Math.PI /2 * (i / (legNum -1) -0.5);
 for (var ii =0; ii < jointNum; ii++) {
 var node = new Segment(
 node,
 size *64 / jointNum,
 ang * (ii ==0),
3.1416,
1.2
 );
 }
 var leg = new LegSystem(node, jointNum, size *30, critter);
 }
 // follow mouse
 targetProvider = function() { return { x: Input.mouse.x, y: Input.mouse.y }; };
}

/**
 * setupLizard
 * - Constructs a creature with a neck, torso, legs, and tail.
 * - size scales overall segment sizes; legs, tail control counts.
 */
function setupLizard(size, legs, tail) {
 var s = size;
 critter = new Creature(
 window.innerWidth /2,
 window.innerHeight /2,
0,
 s *10,
 s *2,
0.5,
16,
0.5,
0.085,
0.5,
0.3
 );
 var spinal = critter;

 // Neck
 for (var i =0; i <6; i++) {
 spinal = new Segment(spinal, s *4,0,3.1415 *2 /3,1.1);
 for (var ii = -1; ii <=1; ii +=2) {
 var node = new Segment(spinal, s *3, ii,0.1,2);
 for (var iii =0; iii <3; iii++) {
 node = new Segment(node, s *0.1, -ii *0.1,0.1,2);
 }
 }
 }

 // Torso and legs
 for (var i =0; i < legs; i++) {
 if (i >0) {
 // Vertebrae and ribs
 for (var ii =0; ii <6; ii++) {
 spinal = new Segment(spinal, s *4,0,1.571,1.5);
 for (var iii = -1; iii <=1; iii +=2) {
 var node = new Segment(spinal, s *3, iii *1.571,0.1,1.5);
 for (var iv =0; iv <3; iv++) {
 node = new Segment(node, s *3, -iii *0.3,0.1,2);
 }
 }
 }
 }
 // Legs and shoulders
 for (var ii = -1; ii <=1; ii +=2) {
 var node = new Segment(spinal, s *12, ii *0.785,0,8); // Hip
 node = new Segment(node, s *16, -ii *0.785,6.28,1); // Humerus
 node = new Segment(node, s *16, ii *1.571,3.1415,2); // Forearm
 for (var iii =0; iii <4; iii++) {
 // Fingers
 new Segment(node, s *4, (iii /3 -0.5) *1.571,0.1,4);
 }
 new LegSystem(node,3, s *12, critter,4);
 }
 }

 // Tail
 for (var i =0; i < tail; i++) {
 spinal = new Segment(spinal, s *4,0,3.1415 *2 /3,1.1);
 for (var ii = -1; ii <=1; ii +=2) {
 var node = new Segment(spinal, s *3, ii,0.1,2);
 for (var iii =0; iii <3; iii++) {
 node = new Segment(node, s *3 * (tail - i) / tail, -ii *0.1,0.1,2);
 }
 }
 }

 // follow mouse
 targetProvider = function() { return { x: Input.mouse.x, y: Input.mouse.y }; };
}

// Visual settings
canvas.style.backgroundColor = "black";
ctx.strokeStyle = "white";
ctx.fillStyle = "white";

// Example initialization — picks a random number of legs and builds a lizard
var legNum = Math.floor(1 + Math.random() *12);
setupLizard(
8 / Math.sqrt(legNum),
 legNum,
 Math.floor(4 + Math.random() * legNum *8)
);

// Main animation loop: single requestAnimationFrame-driven loop with batched drawing
function mainLoop() {
 ctx.clearRect(0,0, canvas.width, canvas.height);

 var t = targetProvider();
 if (critter) {
 // Update simulation
 critter.follow(t.x, t.y);
 // Batched stroke for all segment lines and creature markers
 ctx.beginPath();
 critter.draw(true);
 ctx.stroke();
 }

 // Draw mouse/target indicator (fill, separate path)
 ctx.beginPath();
 ctx.arc(Input.mouse.x, Input.mouse.y,2,0, Math.PI *2);
 ctx.fill();

 requestAnimationFrame(mainLoop);
}

requestAnimationFrame(mainLoop);