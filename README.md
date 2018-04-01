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
 made more precise.

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







