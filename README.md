# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program - Rubric

---

## Compilations

Code Compiles with the regular build tools

## Implementation

### The model

The equations and basic model is taken directly from the lessons. Noteworthy points are:

* speed is sent in mp/h from the simulator. There is a conversion in  main.cpp l.143 to m/s
* latency is taken into account in main.cpp l.178. Not that we can use a simple kinematics model, since we do not have
 any further information about the car / environment anyway (tire diameter, friction, slip, weight, etc.). Also note that
 the latency timing in the project is a bit imprecise, since we do not have realtime OS/communicatin with timing assertions.
 The general latency of the simulator is 25ms-30ms on my machine (measured without the sleep). So setting lateny to 100ms in
 the project will probably result in an effective latency of >100ms. But as mentioned, with Windows/Linux, this cannot be
 made more precis

### Update Explanation of state:

* The actuators are steering angle (from -1 to 1, representing -25 to 25 degrees) and throttle (-1 to 1). They are taken from the
solver in main.cpp l.195/196. Since there is an assymetry in steering angle (simulator sends radians, but expects -1 to -1), it is
adjustes with the division by deg2rad.

* The "physical" state is the car's position (x,y) in simulator map coordinates, its speed (v) and its orientations (psi).  Note that
in a "real" car, other physical forces would be included, e.g. longitudinal and lateral movement/rotation and orientation. Our current model,
e.g., does not deal with slopes, driving on ice. etc.

* The waypoints are interpolated to a polynomial of 3rd degrees, which can describe many road curves (and is, e.g. also used in openDrive).

* The update equations from the latency model are as follows:

          const double px_act = v * latency_in_s;  // Car moves forward along track
          const double py_act = 0; // track is our reference system
          const double psi_act = - v * sa * latency_in_s / Lf;   // formular for adjusting orientation, see section on Lf in class
          const double v_act = v + th * latency_in_s; // v is increased by throttle
          const double cte_act = cte + v * sin(epsi) * latency_in_s;  // errors will change when moving.
          const double epsi_act = epsi + psi_act;

Note that they are simpler than in the lecture, since the coordinat system has been transformed so that the origin is at the car location and
the car orientation is  accccording to the way points, thus not requiring sin/cos operations, except for cte_act, which is the car going off
track.

Basically the same physics are in MPC.cpp, but only for the CppAD library, so that they can be differentiated:

        // Setting up the rest of the model constraints
        fg[1 + x_offs + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_offs + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_offs + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
        fg[1 + v_offs + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_offs + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_offs + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);



### End of Update

* Parameters for MPC are defined at the beginning of MPC.cpp. Note that I replied the type size_t with ulong. size_t is generally
 used for size of datastructures - we just want unsigned longs here.

* Calculations are taken directly from Udacity provided material (lessons, exercises). Major change is the use of const variables instead
 of numerical literals (e.g. in constraints), for easier configurability and consistency. In addition, std::numeric_limits<double>::lowest()
 is being used instead of a hardcoded numeral to support portability.

### N and dt, all parameters

* The implementation was first tested without latency. It worked reasonably fine. Activating latency resulted in an oscillating car. Various tuning
  of parameters was tested, not with satisfactory results. Considering N and dt, the time horizon was just 1 second. Experimenting with the time horizon,
  it seemed reasonable to optimize for a longer distance. Increasing N would result in more calculation effort, so dt was increased.
  A value of 0.4 yielded quite good results.







