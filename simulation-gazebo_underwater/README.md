# Simulation of water effects for underwater systems in Gazebo

This plugin simulates the effect of water on an underwater system within
Gazebo. To use, add a `<plugin …>` block to each model that you'd like to see
affected. The block must be of the form:

~~~
<plugin name="GazeboUnderwater" filename="libgazebo_underwater.so">
</plugin>
~~~

The `name` attribute can be arbitrary, but should be unique. The filename
should be exactly as shown.

The plugin affects the model it's defined in. The following block shows you all
available parameters, as well as their defaults:

~~~
<plugin name="GazeboUnderwater" filename="libgazebo_underwater.so">
  <!-- Water level in meters -->
  <water_level>0</water_level>
  <!-- Resulting buoyancy in newtons, when fully submerged
       The plugin uses a simple model where the buoyancy force is proportional
       to how much the system's bounding box is submerged -->
  <buoyancy>5</buoyancy>
  <!-- The metacenter offset. Metacenter is where the buoyancy force is
       applied.  This is the displacement between the metacenter and the
       CoG (the default is 0.15m upwards -->
  <center_of_buoyancy>0 0 0.15</center_of_buoyancy>
  <!-- TODO -->
  <damping_coefficients>
  [50  0  0  0  0  0;
    0 50  0  0  0  0;
    0  0 50  0  0  0;
    0  0  0 50  0  0;
    0  0  0  0 50  0;
    0  0  0  0  0 50]
  [40  0  0  0  0  0;
    0 40  0  0  0  0;
    0  0 40  0  0  0;
    0  0  0 35  0  0;
    0  0  0  0 35  0;
    0  0  0  0  0 35]
  </damping_coefficients>
  <!-- The added inertia (added mass). The top-left block is truly the added
       mass (in kg). The bottom-right block applies to the moments of inertia (kg.m2)
  -->
  <added_inertia>
  [ 0  0  0  0  0  0;
    0  0  0  0  0  0;
    0  0  0  0  0  0;
    0  0  0  0  0  0;
    0  0  0  0  0  0;
    0  0  0  0  0  0]
  </added_inertia>
</plugin>
~~~
