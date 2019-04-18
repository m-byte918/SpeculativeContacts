var dt                  = 1/120; // Physics delta time
var gravity             = 200;   // Gravity on each particle
var airDrag             = 0.8;   // Damping on velocity of particles
var iterations          = 1;     // Speculative solver iterations
var maxContactImpulse   = 100000;
var maxParticleVelocity = 500;

var contacts = [], particles = [], ctx;

function Vec2(x, y) {
    this.x = x;
    this.y = y;
    this.addScale = function (other, speed) {
        this.x += other.x * speed;
        this.y += other.y * speed;
    }
    this.subScale = function (other, speed) {
        this.x -= other.x * speed;
        this.y -= other.y * speed;
    }
    this.difference = function (other) {
        return new Vec2(this.x - other.x, this.y - other.y);
    }
    this.product = function (val) {
        return new Vec2(this.x * val, this.y * val);
    }
    this.dot = function (v) {
        return this.x * v.x + this.y * v.y;
    }
}
function Particle(x, y, radius) {
    this.position = new Vec2(x, y);
    this.velocity = new Vec2(0, 1);
    this.force    = new Vec2(0, 0);
    this.radius   = radius;
    this.invMass  = 1 / (radius * radius / 100);
    this.collidingWith = function (other) {
        var rs = this.radius + other.radius;
        var d = this.position.difference(other.position);
        return d.dot(d) < (rs * rs);
    }
}
function Contact(a, b) {
    this.a           = a;
    this.b           = b;
    this.normal      = new Vec2(0, 1);
    this.impulse     = 0;
    this.penetration = 0;
}

function init() {
    // Init rendering
    ctx = canvas.getContext("2d");
    ctx.strokeStyle = "black";
    ctx.fillStyle = "silver";
    ctx.lineWidth = 1;

    // Start loop
    setInterval(update, dt * 1000);
}
function update() {
    particles.forEach(p => {
        // Reset force
        p.force.x = 0;
        p.force.y = 0;

        // Add gravity
        p.force.y += gravity / p.invMass;

        // Add spatial damping
        var damping = Math.pow(airDrag, dt);
        p.velocity.x *= damping;
        p.velocity.y *= damping;

        // Restrict velocity
        var speed = Math.sqrt(p.velocity.dot(p.velocity));
        if (Math.round(speed)) {
            p.velocity.x /= speed; // Normalize
            p.velocity.y /= speed; // Normalize
            speed = Math.max(-maxParticleVelocity, Math.min(speed, maxParticleVelocity)); // Clamp
            p.velocity.x *= speed;
            p.velocity.y *= speed;
        }
    });
    // Generate contacts
    contacts = [];
    particles.forEach(b => {
        // Using brute force for now (gross, I know)
        particles.forEach(a => {
            if (a === b || !a.collidingWith(b))
                return;
            var c = new Contact(a, b);
            var diff = b.position.difference(a.position);
            var dist = Math.sqrt(diff.dot(diff));
            // Use normal from previous position
            if (dist) {
                c.normal.x    = diff.x / dist;
                c.normal.y    = diff.y / dist;
                c.penetration = (b.radius + a.radius) - dist;
            } else {
                c.penetration = a.radius;
            }
            contacts.push(c);
        });
    });
    // Solve contacts
    for (var it = 0; it < iterations; ++it) {
        for (var i = contacts.length - 1; i >= 0; --i) {
            var con = contacts[i];

            // Get all of relative normal velocity
            var relNormalVelocity = con.b.velocity.difference(con.a.velocity).dot(con.normal);
            if (relNormalVelocity > 0) continue;

            // Remove all relative velocity + leave them touching after this time step
            var relForce  = con.b.force.product(con.b.invMass).difference(con.a.force.product(con.a.invMass)).dot(con.normal);
            var removeVel = relNormalVelocity + dt * relForce - con.penetration / dt;
            var invSum    = con.a.invMass + con.b.invMass;
            var imp       = removeVel / invSum;

            // Restrict impulse
            var newImpulse = Math.max(-maxContactImpulse, Math.min(imp + con.impulse, maxContactImpulse));
            var change     = newImpulse - con.impulse;
            con.impulse    = newImpulse;

            // Apply impulse
            newImpulse = con.normal.product(change);
            con.a.velocity.addScale(newImpulse, con.a.invMass);
            con.b.velocity.subScale(newImpulse, con.b.invMass);

            // Positional correction
            var percent = 0.2;
            var slop    = 0.01;
            change = con.normal.product((Math.max(con.penetration - slop, 0) / invSum) * percent);
            con.a.position.addScale(change, con.a.invMass);
            con.b.position.subScale(change, con.b.invMass);
        }
    }
    // Integrate forces
    particles.forEach(p => {
        p.velocity.addScale(p.force.product(dt), p.invMass);
        p.position.addScale(p.velocity, dt);

        // Bounce off of canvas border
        if (p.position.x + p.radius >= canvas.width) {
            p.position.x = canvas.width - p.radius;
            p.velocity.x = -p.velocity.x;
        }
        if (p.position.x - p.radius <= 0) {
            p.position.x = p.radius;
            p.velocity.x = -p.velocity.x;
        }
        if (p.position.y + p.radius >= canvas.height) {
            p.position.y = canvas.height - p.radius;
            p.velocity.y = -p.velocity.y;
        }
        if (p.position.y - p.radius <= 0) {
            p.position.y = p.radius;
            p.velocity.y = -p.velocity.y;
        }
    });
    // Render
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.strokeRect(0, 0, canvas.width, canvas.height);
    particles.forEach(p => {
        ctx.beginPath();
        ctx.arc(p.position.x, p.position.y, p.radius, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.fill();
    });
    // Check if contacts were solved
    var solvedContacts = 0;
    contacts.forEach(con => {
        if (!con.a.collidingWith(con.b)) {
            ++solvedContacts;
            return;
        }
        // Draw line between contacts
        ctx.beginPath();
        ctx.moveTo(con.a.position.x + (con.a.radius / 1.5 * con.normal.x), 
            con.a.position.y + (con.a.radius / 1.5 * con.normal.y));
        ctx.lineTo(con.b.position.x - (con.b.radius / 1.5 * con.normal.x), 
            con.b.position.y - (con.b.radius / 1.5 * con.normal.y));
        ctx.stroke();
    });
    document.getElementById('iterations').innerHTML = "Solver iterations: " + iterations;
    document.getElementById('particles').innerHTML = "Particles: " + particles.length;
    document.getElementById('contacts').innerHTML = "Contacts: " + contacts.length;
    document.getElementById('solvedContacts').innerHTML = "Contacts solved in this timestep: " + solvedContacts;
}
// Controls
var mouse = new Vec2(0, 0);
var lastAirDrag = airDrag;
var lastGravity = gravity;
window.addEventListener('keydown', function (e) {
    e.preventDefault();
    var r = 0;
    switch (e.keyCode) {
        // Backspace
        case 8: {
            contacts = [];
            particles = [];
            return;
        }
        // Enter
        case 13: {
            airDrag = airDrag == 0 ? lastAirDrag : 0;
            gravity = gravity == 0 ? lastGravity : 0;
            return;
        }
        // Up arrow
        case 38: {
            ++iterations;
            return;
        }
        // Down arrow
        case 40: {
            iterations = Math.max(1, iterations - 1);
            return;
        }
        case 49: { r = 8;  break; } // 1
        case 50: { r = 10; break; } // 2
        case 51: { r = 12; break; } // 3
        case 52: { r = 14; break; } // 4
        case 53: { r = 16; break; } // 5
        case 54: { r = 18; break; } // 6
        case 55: { r = 20; break; } // 7
        case 56: { r = 22; break; } // 8
        case 57: { r = 24; break; } // 9
        default: return;
    };
    particles.push(new Particle(mouse.x, mouse.y, r));
}, true);
// Update mouse coordinates
canvas.addEventListener('mousemove', function (e) {
    var rect = canvas.getBoundingClientRect();
    mouse.x = e.clientX - rect.left;
    mouse.y = e.clientY - rect.top;
});
init();
