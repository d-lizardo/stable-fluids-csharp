using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FluidSimulation
{
    /// <summary>
    /// A C# implementation of Jos Stam's Stable Fluids algorithm
    /// Adapted for use in Rhino/Grasshopper plugins
    /// </summary>
    public class StableFluidSimulation
    {
        private readonly int width, height;
        private readonly float dt;
        private readonly float viscosity;
        private readonly float diffusion;
        private readonly int solverIterations;

        // Velocity fields (current and previous)
        private float[,] u, v;           // Current velocity
        private float[,] u_prev, v_prev; // Previous velocity

        // Density field for visualization
        private float[,] density, density_prev;

        // Temporary arrays for solver
        private float[,] temp1, temp2;

        public StableFluidSimulation(int width, int height, float timeStep = 0.016f,
                                   float viscosity = 0.0001f, float diffusion = 0.0001f,
                                   int solverIterations = 10)
        {
            this.width = width;
            this.height = height;
            this.dt = timeStep;
            this.viscosity = viscosity;
            this.diffusion = diffusion;
            this.solverIterations = solverIterations;
            
            // Initialize arrays
            u = new float[width, height];
            v = new float[width, height];
            u_prev = new float[width, height];
            v_prev = new float[width, height];
            
            density = new float[width, height];
            density_prev = new float[width, height];
            
            temp1 = new float[width, height];
            temp2 = new float[width, height];
        }

        /// <summary>
        /// Add force at a specific location
        /// </summary>
        public void AddForce(int x, int y, float forceX, float forceY)
        {
            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                u[x, y] += forceX;
                v[x, y] += forceY;
            }
        }

        /// <summary>
        /// Add density at a specific location (for visualization)
        /// </summary>
        public void AddDensity(int x, int y, float amount)
        {
            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                density[x, y] += amount;
            }
        }

        /// <summary>
        /// Perform one simulation step
        /// </summary>
        public void Step()
        {
            // Velocity step
            VelocityStep();
            
            // Density step (for visualization)
            DensityStep();
        }

        private void VelocityStep()
        {
            // Add forces (already done via AddForce method)
            
            // Swap arrays
            Swap(ref u, ref u_prev);
            Swap(ref v, ref v_prev);
            
            // Diffusion
            Diffuse(1, u, u_prev, viscosity, dt);
            Diffuse(2, v, v_prev, viscosity, dt);
            
            // Project to make divergence-free
            Project(u, v, u_prev, v_prev);
            
            // Swap again
            Swap(ref u, ref u_prev);
            Swap(ref v, ref v_prev);
            
            // Advection
            Advect(1, u, u_prev, u_prev, v_prev, dt);
            Advect(2, v, v_prev, u_prev, v_prev, dt);
            
            // Project again
            Project(u, v, u_prev, v_prev);
        }

        private void DensityStep()
        {
            Swap(ref density, ref density_prev);
            
            // Diffuse density
            Diffuse(0, density, density_prev, diffusion, dt);
            
            Swap(ref density, ref density_prev);
            
            // Advect density
            Advect(0, density, density_prev, u, v, dt);
        }

        /// <summary>
        /// Diffusion step using Gauss-Seidel relaxation
        /// </summary>
        private void Diffuse(int b, float[,] x, float[,] x0, float diff, float dt)
        {
            float a = dt * diff * (width - 2) * (height - 2);
            LinearSolve(b, x, x0, a, 1 + 4 * a);
        }

        /// <summary>
        /// Linear solver using Gauss-Seidel relaxation with parallelization
        /// </summary>
        private void LinearSolve(int b, float[,] x, float[,] x0, float a, float c)
        {
            float cRecip = 1.0f / c;

            // Iterate to solve the linear system (reduced iterations for speed)
            for (int k = 0; k < solverIterations; k++)
            {
                // Use red-black Gauss-Seidel for parallelization
                // Red cells (i+j is even)
                Parallel.For(1, height - 1, j =>
                {
                    for (int i = 1; i < width - 1; i++)
                    {
                        if ((i + j) % 2 == 0)
                        {
                            x[i, j] = (x0[i, j] + a * (x[i + 1, j] + x[i - 1, j] +
                                                       x[i, j + 1] + x[i, j - 1])) * cRecip;
                        }
                    }
                });

                // Black cells (i+j is odd)
                Parallel.For(1, height - 1, j =>
                {
                    for (int i = 1; i < width - 1; i++)
                    {
                        if ((i + j) % 2 == 1)
                        {
                            x[i, j] = (x0[i, j] + a * (x[i + 1, j] + x[i - 1, j] +
                                                       x[i, j + 1] + x[i, j - 1])) * cRecip;
                        }
                    }
                });

                SetBoundary(b, x);
            }
        }

        /// <summary>
        /// Advection step using semi-Lagrangian method with parallelization
        /// </summary>
        private void Advect(int b, float[,] d, float[,] d0, float[,] velocX, float[,] velocY, float dt)
        {
            float dtx = dt * (width - 2);
            float dty = dt * (height - 2);
            float Nfloat = width - 2;
            float Mfloat = height - 2;

            // Parallelize the outer loop
            Parallel.For(1, height - 1, j =>
            {
                for (int i = 1; i < width - 1; i++)
                {
                    float tmp1 = dtx * velocX[i, j];
                    float tmp2 = dty * velocY[i, j];
                    float x = i - tmp1;
                    float y = j - tmp2;

                    if (x < 0.5f) x = 0.5f;
                    if (x > Nfloat + 0.5f) x = Nfloat + 0.5f;
                    float i0 = (float)Math.Floor(x);
                    float i1 = i0 + 1.0f;

                    if (y < 0.5f) y = 0.5f;
                    if (y > Mfloat + 0.5f) y = Mfloat + 0.5f;
                    float j0 = (float)Math.Floor(y);
                    float j1 = j0 + 1.0f;

                    float s1 = x - i0;
                    float s0 = 1.0f - s1;
                    float t1 = y - j0;
                    float t0 = 1.0f - t1;

                    int i0i = (int)i0;
                    int i1i = (int)i1;
                    int j0i = (int)j0;
                    int j1i = (int)j1;

                    d[i, j] = s0 * (t0 * d0[i0i, j0i] + t1 * d0[i0i, j1i]) +
                              s1 * (t0 * d0[i1i, j0i] + t1 * d0[i1i, j1i]);
                }
            });

            SetBoundary(b, d);
        }

        /// <summary>
        /// Projection step to ensure divergence-free velocity field with parallelization
        /// </summary>
        private void Project(float[,] velocX, float[,] velocY, float[,] p, float[,] div)
        {
            // Calculate divergence (parallelized)
            Parallel.For(1, height - 1, j =>
            {
                for (int i = 1; i < width - 1; i++)
                {
                    div[i, j] = -0.5f * (velocX[i + 1, j] - velocX[i - 1, j] +
                                         velocY[i, j + 1] - velocY[i, j - 1]) / width;
                    p[i, j] = 0;
                }
            });

            SetBoundary(0, div);
            SetBoundary(0, p);
            LinearSolve(0, p, div, 1, 4);

            // Subtract pressure gradient (parallelized)
            Parallel.For(1, height - 1, j =>
            {
                for (int i = 1; i < width - 1; i++)
                {
                    velocX[i, j] -= 0.5f * (p[i + 1, j] - p[i - 1, j]) * width;
                    velocY[i, j] -= 0.5f * (p[i, j + 1] - p[i, j - 1]) * width;
                }
            });

            SetBoundary(1, velocX);
            SetBoundary(2, velocY);
        }

        /// <summary>
        /// Set boundary conditions
        /// </summary>
        private void SetBoundary(int b, float[,] x)
        {
            for (int i = 1; i < height - 1; i++)
            {
                x[0, i] = b == 1 ? -x[1, i] : x[1, i];
                x[width - 1, i] = b == 1 ? -x[width - 2, i] : x[width - 2, i];
            }
            
            for (int i = 1; i < width - 1; i++)
            {
                x[i, 0] = b == 2 ? -x[i, 1] : x[i, 1];
                x[i, height - 1] = b == 2 ? -x[i, height - 2] : x[i, height - 2];
            }

            x[0, 0] = 0.5f * (x[1, 0] + x[0, 1]);
            x[0, height - 1] = 0.5f * (x[1, height - 1] + x[0, height - 2]);
            x[width - 1, 0] = 0.5f * (x[width - 2, 0] + x[width - 1, 1]);
            x[width - 1, height - 1] = 0.5f * (x[width - 2, height - 1] + x[width - 1, height - 2]);
        }

        private void Swap(ref float[,] a, ref float[,] b)
        {
            var temp = a;
            a = b;
            b = temp;
        }

        // Public accessors for Grasshopper
        public float[,] VelocityU => u;
        public float[,] VelocityV => v;
        public float[,] Density => density;
        
        public float GetVelocityMagnitude(int x, int y)
        {
            if (x >= 0 && x < width && y >= 0 && y < height)
                return (float)Math.Sqrt(u[x, y] * u[x, y] + v[x, y] * v[x, y]);
            return 0f;
        }

        /// <summary>
        /// Clear all forces from the velocity field (but preserve existing velocity)
        /// This should be called before applying new forces for each step
        /// Note: Since forces are applied directly to the velocity field in AddForce,
        /// we can't easily separate "forces" from "velocity". This method is a placeholder
        /// for a more sophisticated force management system.
        /// </summary>
        public void ClearForces()
        {
            // In the current implementation, forces are applied directly to the velocity field
            // So there's no separate "force" field to clear. This is intentionally empty.
            // For a more realistic simulation, you might want to implement a separate force
            // accumulation system that gets applied and then cleared each step.
        }

        /// <summary>
        /// Execute a motion profile step by step up to a given fraction
        /// This simulates dragging an object through fluid
        /// </summary>
        public void ExecuteMotionProfile(MotionProfile profile, float fraction, float forceScale = 10.0f,
                                        double worldWidth = 100.0, double worldHeight = 100.0)
        {
            if (profile == null || profile.Steps.Count == 0) return;

            int maxStep = (int)(profile.Steps.Count * Math.Max(0, Math.Min(1, fraction)));

            for (int i = 0; i < maxStep; i++)
            {
                var step = profile.Steps[i];

                // Convert world coordinates to grid coordinates
                double normalizedX = step.Position.X / worldWidth;
                double normalizedY = step.Position.Y / worldHeight;
                int gridX = (int)Math.Max(0, Math.Min(width - 1, normalizedX * width));
                int gridY = (int)Math.Max(0, Math.Min(height - 1, normalizedY * height));

                // Normalize velocity to grid scale
                double velocityScaleX = width / worldWidth;
                double velocityScaleY = height / worldHeight;

                // Apply impulse force based on velocity (tangent direction)
                // This represents the force of dragging an object through the fluid
                float forceX = (float)(step.Velocity.X * velocityScaleX * forceScale);
                float forceY = (float)(step.Velocity.Y * velocityScaleY * forceScale);

                // Apply force in a small area around the point to make it more realistic
                int radius = 2; // Apply force in a 5x5 area
                for (int dx = -radius; dx <= radius; dx++)
                {
                    for (int dy = -radius; dy <= radius; dy++)
                    {
                        int fx = gridX + dx;
                        int fy = gridY + dy;
                        if (fx >= 0 && fx < width && fy >= 0 && fy < height)
                        {
                            // Reduce force with distance from center
                            float distance = (float)Math.Sqrt(dx * dx + dy * dy);
                            float falloff = Math.Max(0, 1.0f - distance / (radius + 1));
                            AddForce(fx, fy, forceX * falloff, forceY * falloff);
                        }
                    }
                }

                // Add density to visualize the path
                AddDensity(gridX, gridY, 30.0f);

                // Step the simulation forward
                Step();
            }
        }
    }

    /// <summary>
    /// Represents a single step in a motion profile
    /// </summary>
    public class MotionStep
    {
        public Point3d Position { get; set; }
        public Vector3d Velocity { get; set; }
        public float Time { get; set; }

        public MotionStep(Point3d position, Vector3d velocity, float time)
        {
            Position = position;
            Velocity = velocity;
            Time = time;
        }
    }

    /// <summary>
    /// Represents a complete motion profile for fluid simulation
    /// Contains a sequence of positions and velocities over time
    /// </summary>
    public class MotionProfile
    {
        public List<MotionStep> Steps { get; private set; }
        public float TotalTime { get; private set; }

        public MotionProfile()
        {
            Steps = new List<MotionStep>();
            TotalTime = 0f;
        }

        public MotionProfile(List<Point3d> positions, List<Vector3d> velocities, float timeStep)
        {
            Steps = new List<MotionStep>();
            TotalTime = 0f;

            int count = Math.Min(positions.Count, velocities.Count);
            for (int i = 0; i < count; i++)
            {
                var step = new MotionStep(positions[i], velocities[i], i * timeStep);
                Steps.Add(step);
                TotalTime = step.Time;
            }
        }

        public void AddStep(Point3d position, Vector3d velocity, float time)
        {
            Steps.Add(new MotionStep(position, velocity, time));
            TotalTime = Math.Max(TotalTime, time);
        }

        public int StepCount => Steps.Count;
    }
}
