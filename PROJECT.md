# Project Plan and Notes

~~1. Set up simulation based on Stable Fluids~~

2. Convert dimensionless outputs to full scale

3. Switch from step-activated to pre-determined number of simulation steps

4. Create motion "profile" from curve inputs -> force point, force vector, density point, density amount should\
all be calculated based on curves and speed of traversal.
    4.1 divide curve by timestep ( length = speed * time )
    4.2 sync timestep in curve speed and simulation step
    4.3 vector = curve tangent at curve point
    4.4 vector magnitude... something static <- this is likely a tunable parameter.

5. Convert data/"step" inputs to "steps" table and run through all steps.

6. Slider for percentage of steps completed so you can roll the simulation forward and backwards.