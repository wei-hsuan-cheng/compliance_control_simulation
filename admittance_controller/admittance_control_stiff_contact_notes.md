# 1-DoF Admittance Control With Stiff Environment Contact

This note describes a simple 1-DoF simulation model for an outer admittance loop with an inner position servo loop.

The intended architecture is:

\[
x_r(t)
\rightarrow
\text{outer admittance}
\rightarrow
x_c(t)
\rightarrow
\text{inner position servo}
\rightarrow
x(t)
\]

where:

- \(x_r(t)\): nominal teleop / leader / planner reference.
- \(\Delta x(t)\): admittance-generated pose deviation.
- \(x_c(t)\): modified command sent to the robot position controller.
- \(x(t)\): actual end-effector position.
- \(x_w\): wall/environment position.
- \(F_{env}\): measured environment contact force.
- The inner position servo is assumed to be perfect.

Therefore:

\[
x_c = x_r + \Delta x
\]

and with perfect low-level tracking:

\[
\boxed{x = x_c = x_r + \Delta x}
\]

---

## 1. Sign Convention

Assume positive \(x\) points **toward / into the wall**.

The wall is located at:

\[
x = x_w
\]

The penetration depth is:

\[
\boxed{\delta = x - x_w}
\]

So:

\[
\delta \leq 0
\quad\Rightarrow\quad
\text{no contact}
\]

\[
\delta > 0
\quad\Rightarrow\quad
\text{contact}
\]

During contact, the wall pushes the robot backward, so:

\[
F_{env} < 0
\]

---

## 2. Outer Admittance Loop

The admittance loop is applied to the deviation \(\Delta x\), not directly to the robot position.

\[
\boxed{
M_a \Delta \ddot{x}
+
D_a \Delta \dot{x}
+
K_a \Delta x
=
F_{env}
}
\]

where:

- \(M_a\): virtual admittance inertia.
- \(D_a\): virtual admittance damping.
- \(K_a\): virtual admittance stiffness.
- \(F_{env}\): measured environment force.

Solving for acceleration:

\[
\boxed{
\Delta \ddot{x}
=
\frac{
F_{env}
-
D_a\Delta \dot{x}
-
K_a\Delta x
}{M_a}
}
\]

This is the acceleration-driven simulation equation.

Then integrate twice:

\[
\Delta \dot{x}_{k+1}
=
\Delta \dot{x}_{k}
+
\Delta \ddot{x}_{k} \Delta t
\]

\[
\Delta x_{k+1}
=
\Delta x_k
+
\Delta \dot{x}_{k+1}\Delta t
\]

This is the **semi-implicit Euler** update because the new velocity is used to update position.

---

## 3. Environment Model

The actual robot position is:

\[
x = x_r + \Delta x
\]

The actual robot velocity is:

\[
\dot{x} = \dot{x}_r + \Delta \dot{x}
\]

For a fixed wall:

\[
\dot{x}_w = 0
\]

The penetration velocity is:

\[
\dot{\delta}
=
\dot{x} - \dot{x}_w
=
\dot{x}_r + \Delta \dot{x}
\]

A Kelvin-Voigt wall model is:

\[
F_{env}
=
-K_e \delta
-
D_e \dot{\delta}
\]

where:

- \(K_e\): environment stiffness.
- \(D_e\): environment damping.

However, because contact is unilateral, the force should not pull the robot into the wall. A practical implementation is:

\[
\boxed{
F_{env}
=
\begin{cases}
0, & \delta \leq 0 \\
\min\left(0,\ -K_e\delta - D_e\dot{\delta}\right), & \delta > 0
\end{cases}
}
\]

The `min(0, ...)` avoids tensile contact force.

---

## 4. Combined Contact Dynamics

During active contact:

\[
M_a\Delta \ddot{x}
+
D_a\Delta \dot{x}
+
K_a\Delta x
=
-K_e(x_r+\Delta x-x_w)
-
D_e(\dot{x}_r+\Delta \dot{x})
\]

Move terms to the left:

\[
\boxed{
M_a\Delta \ddot{x}
+
(D_a+D_e)\Delta \dot{x}
+
(K_a+K_e)\Delta x
=
-K_e(x_r-x_w)
-
D_e\dot{x}_r
}
\]

This is the combined closed-loop equation in contact.

The effective contact-mode parameters are:

\[
D_{eff} = D_a + D_e
\]

\[
K_{eff} = K_a + K_e
\]

The contact natural frequency is:

\[
\boxed{
\omega_n
=
\sqrt{\frac{K_a+K_e}{M_a}}
}
\]

The contact damping ratio is:

\[
\boxed{
\zeta
=
\frac{D_a+D_e}
{2\sqrt{M_a(K_a+K_e)}}
}
\]

The eigenvalues are:

\[
\boxed{
\lambda_{1,2}
=
\frac{
-(D_a+D_e)
\pm
\sqrt{(D_a+D_e)^2-4M_a(K_a+K_e)}
}
{2M_a}
}
\]

---

## 5. Behavior Before and After Contact

### No contact

If:

\[
x_r + \Delta x \leq x_w
\]

then:

\[
F_{env}=0
\]

The admittance dynamics become:

\[
M_a\Delta \ddot{x}
+
D_a\Delta \dot{x}
+
K_a\Delta x
=
0
\]

If initialized with:

\[
\Delta x(0)=0,\qquad \Delta \dot{x}(0)=0
\]

then:

\[
\Delta x(t)=0
\]

therefore:

\[
\boxed{x(t)=x_r(t)}
\]

So the robot follows the teleop nominal reference perfectly before contact.

---

### Contact

If:

\[
x_r+\Delta x > x_w
\]

then:

\[
F_{env}<0
\]

and the admittance loop generates:

\[
\Delta x < 0
\]

So the actual command becomes:

\[
x = x_r+\Delta x < x_r
\]

The robot yields backward from the nominal reference.

---

## 6. Steady-State Penetration for a Stopped Reference

Assume the teleop reference stops slightly inside the wall:

\[
x_r = x_w + \epsilon
\]

where:

\[
\epsilon > 0
\]

At steady state:

\[
\Delta \dot{x}=0,\qquad
\Delta \ddot{x}=0,\qquad
\dot{x}_r=0
\]

The contact equation becomes:

\[
K_a\Delta x
=
-K_e(x_r+\Delta x-x_w)
\]

Since:

\[
x_r-x_w=\epsilon
\]

we get:

\[
K_a\Delta x
=
-K_e(\epsilon+\Delta x)
\]

\[
(K_a+K_e)\Delta x=-K_e\epsilon
\]

Therefore:

\[
\boxed{
\Delta x_{ss}
=
-\frac{K_e}{K_a+K_e}\epsilon
}
\]

The steady-state actual penetration is:

\[
\delta_{ss}
=
x_{ss}-x_w
=
x_r+\Delta x_{ss}-x_w
\]

\[
\boxed{
\delta_{ss}
=
\frac{K_a}{K_a+K_e}\epsilon
}
\]

The steady-state contact force is:

\[
F_{env,ss}
=
-K_e\delta_{ss}
\]

\[
\boxed{
F_{env,ss}
=
-\frac{K_aK_e}{K_a+K_e}\epsilon
}
\]

If the wall is very stiff:

\[
K_e \gg K_a
\]

then:

\[
\boxed{
\delta_{ss}\approx \frac{K_a}{K_e}\epsilon
}
\]

and:

\[
\boxed{
F_{env,ss}\approx -K_a\epsilon
}
\]

This means that for a very stiff wall, the final force is mostly limited by the admittance stiffness \(K_a\).

---

## 7. Tuning Insights

### \(M_a\): virtual inertia

Larger \(M_a\):

- Slower acceleration response.
- Less sensitive to force spikes.
- Lower contact natural frequency.
- Can become more oscillatory if \(D_a\) is not increased.

\[
\omega_n = \sqrt{\frac{K_a+K_e}{M_a}}
\]

---

### \(D_a\): virtual damping

Larger \(D_a\):

- More stable contact.
- Less overshoot and chatter.
- More dissipative.
- Lower transparency / more sluggish yielding.

\[
\zeta =
\frac{D_a+D_e}
{2\sqrt{M_a(K_a+K_e)}}
\]

For stiff contact, \(D_a\) is usually the most important stabilizing knob.

---

### \(K_a\): virtual stiffness

Larger \(K_a\):

- Smaller deviation from nominal \(x_r\).
- Higher steady contact force if \(x_r\) is commanded into the wall.
- Higher contact natural frequency.
- More likely to excite stiff-contact oscillation.

For very stiff walls:

\[
F_{env,ss}\approx -K_a(x_r-x_w)
\]

So \(K_a\) controls the final contact force.

---

### \(K_e\): environment stiffness

Larger \(K_e\):

- Harder wall.
- Higher contact natural frequency.
- Lower damping ratio if \(D_a\) is unchanged.
- Smaller physical penetration for the same contact force.

\[
K_e \uparrow
\Rightarrow
\omega_n \uparrow
\]

\[
K_e \uparrow
\Rightarrow
\zeta \downarrow
\]

---

### \(D_e\): environment damping

Larger \(D_e\):

- More physical damping at the contact.
- Reduces impact velocity.
- Helps stabilize contact.
- But in simulation, the contact force should usually be clamped to avoid tensile force.

---

## 8. Discrete-Time Simulation Algorithm

At each timestep:

1. Get nominal teleop reference:

\[
x_r[k],\quad \dot{x}_r[k]
\]

2. Compute actual robot state under perfect inner position tracking:

\[
x[k]=x_r[k]+\Delta x[k]
\]

\[
\dot{x}[k]=\dot{x}_r[k]+\Delta \dot{x}[k]
\]

3. Compute penetration:

\[
\delta[k]=x[k]-x_w
\]

\[
\dot{\delta}[k]=\dot{x}[k]
\]

4. Compute environment force:

\[
F_{env}[k]
=
\begin{cases}
0, & \delta[k]\leq0 \\
\min(0,\ -K_e\delta[k]-D_e\dot{\delta}[k]), & \delta[k]>0
\end{cases}
\]

5. Compute admittance acceleration:

\[
\Delta \ddot{x}[k]
=
\frac{
F_{env}[k]
-
D_a\Delta \dot{x}[k]
-
K_a\Delta x[k]
}{M_a}
\]

6. Semi-implicit Euler integration:

\[
\Delta \dot{x}[k+1]
=
\Delta \dot{x}[k]
+
\Delta \ddot{x}[k]\Delta t
\]

\[
\Delta x[k+1]
=
\Delta x[k]
+
\Delta \dot{x}[k+1]\Delta t
\]

7. Updated command:

\[
x_c[k+1]
=
x_r[k+1]+\Delta x[k+1]
\]

8. Perfect inner servo:

\[
x[k+1]=x_c[k+1]
\]

---

## 9. Important Modeling Assumption

This model assumes the inner position servo is perfect:

\[
x=x_c
\]

This removes the robot's actuator dynamics, tracking delay, saturation, friction, and compliance.

In real stiff contact, those ignored effects are often what destabilize admittance control. A more realistic model would add an inner position-servo dynamics model such as:

\[
\ddot{x}
=
\omega_s^2(x_c-x)
-
2\zeta_s\omega_s\dot{x}
\]

or include discrete delay, command saturation, and force filtering.

But for first-level understanding, the perfect-servo model is a good starting point.
