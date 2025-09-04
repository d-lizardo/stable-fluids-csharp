using System;
using System.Drawing;
using System.Collections.Generic;

namespace FluidSimulation
{
    /// <summary>
    /// Helper class for visualizing fluid simulation results
    /// Generates bitmap images and vector field data for Grasshopper
    /// </summary>
    public class FluidVisualization
    {
        private readonly StableFluidSimulation simulation;
        private readonly int width, height;

        public FluidVisualization(StableFluidSimulation simulation)
        {
            this.simulation = simulation;
            this.width = simulation.VelocityU.GetLength(0);
            this.height = simulation.VelocityU.GetLength(1);
        }

        /// <summary>
        /// Generate a bitmap showing the velocity field magnitude
        /// </summary>
        public Bitmap CreateVelocityMagnitudeBitmap()
        {
            var bitmap = new Bitmap(width, height);
            float maxMagnitude = GetMaxVelocityMagnitude();
            
            if (maxMagnitude == 0) maxMagnitude = 1; // Avoid division by zero

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    float magnitude = simulation.GetVelocityMagnitude(x, y);
                    float normalized = Math.Min(magnitude / maxMagnitude, 1.0f);
                    
                    // Create a color from blue (low) to red (high)
                    int intensity = (int)(normalized * 255);
                    Color color = Color.FromArgb(intensity, 0, 255 - intensity);
                    
                    bitmap.SetPixel(x, height - 1 - y, color); // Flip Y for correct orientation
                }
            }
            
            return bitmap;
        }

        /// <summary>
        /// Generate a bitmap showing the density field
        /// </summary>
        public Bitmap CreateDensityBitmap()
        {
            var bitmap = new Bitmap(width, height);
            float maxDensity = GetMaxDensity();
            
            if (maxDensity == 0) maxDensity = 1;

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    float density = simulation.Density[x, y];
                    float normalized = Math.Min(density / maxDensity, 1.0f);
                    
                    int intensity = (int)(normalized * 255);
                    Color color = Color.FromArgb(intensity, intensity, intensity);
                    
                    bitmap.SetPixel(x, height - 1 - y, color);
                }
            }
            
            return bitmap;
        }

        /// <summary>
        /// Generate streamlines for vector field visualization
        /// </summary>
        public List<List<PointF>> GenerateStreamlines(int numStreamlines = 20, int maxSteps = 100, float stepSize = 0.5f)
        {
            var streamlines = new List<List<PointF>>();
            var random = new Random();

            for (int i = 0; i < numStreamlines; i++)
            {
                // Random starting point
                float startX = random.Next(1, width - 1);
                float startY = random.Next(1, height - 1);
                
                var streamline = TraceStreamline(startX, startY, maxSteps, stepSize);
                if (streamline.Count > 1)
                {
                    streamlines.Add(streamline);
                }
            }

            return streamlines;
        }

        /// <summary>
        /// Trace a single streamline from a starting point
        /// </summary>
        private List<PointF> TraceStreamline(float startX, float startY, int maxSteps, float stepSize)
        {
            var points = new List<PointF>();
            float x = startX, y = startY;

            for (int step = 0; step < maxSteps; step++)
            {
                // Check bounds
                if (x < 1 || x >= width - 1 || y < 1 || y >= height - 1)
                    break;

                points.Add(new PointF(x, y));

                // Get velocity at current position (bilinear interpolation)
                var velocity = GetInterpolatedVelocity(x, y);
                
                // Check if velocity is too small
                float magnitude = (float)Math.Sqrt(velocity.X * velocity.X + velocity.Y * velocity.Y);
                if (magnitude < 0.001f)
                    break;

                // Move to next position
                x += velocity.X * stepSize;
                y += velocity.Y * stepSize;
            }

            return points;
        }

        /// <summary>
        /// Get interpolated velocity at a floating-point position
        /// </summary>
        private PointF GetInterpolatedVelocity(float x, float y)
        {
            int x0 = (int)Math.Floor(x);
            int y0 = (int)Math.Floor(y);
            int x1 = Math.Min(x0 + 1, width - 1);
            int y1 = Math.Min(y0 + 1, height - 1);

            float fx = x - x0;
            float fy = y - y0;

            // Bilinear interpolation
            float u00 = simulation.VelocityU[x0, y0];
            float u10 = simulation.VelocityU[x1, y0];
            float u01 = simulation.VelocityU[x0, y1];
            float u11 = simulation.VelocityU[x1, y1];

            float v00 = simulation.VelocityV[x0, y0];
            float v10 = simulation.VelocityV[x1, y0];
            float v01 = simulation.VelocityV[x0, y1];
            float v11 = simulation.VelocityV[x1, y1];

            float u = u00 * (1 - fx) * (1 - fy) + u10 * fx * (1 - fy) + 
                      u01 * (1 - fx) * fy + u11 * fx * fy;
            float v = v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy) + 
                      v01 * (1 - fx) * fy + v11 * fx * fy;

            return new PointF(u, v);
        }

        /// <summary>
        /// Generate a regular grid of velocity vectors for visualization
        /// </summary>
        public List<VelocityVector> GetVelocityVectors(int gridSpacing = 5)
        {
            var vectors = new List<VelocityVector>();

            for (int x = gridSpacing; x < width - gridSpacing; x += gridSpacing)
            {
                for (int y = gridSpacing; y < height - gridSpacing; y += gridSpacing)
                {
                    float u = simulation.VelocityU[x, y];
                    float v = simulation.VelocityV[x, y];
                    float magnitude = (float)Math.Sqrt(u * u + v * v);

                    if (magnitude > 0.001f) // Only include significant vectors
                    {
                        vectors.Add(new VelocityVector
                        {
                            X = x,
                            Y = y,
                            U = u,
                            V = v,
                            Magnitude = magnitude
                        });
                    }
                }
            }

            return vectors;
        }

        private float GetMaxVelocityMagnitude()
        {
            float max = 0;
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    float magnitude = simulation.GetVelocityMagnitude(x, y);
                    if (magnitude > max) max = magnitude;
                }
            }
            return max;
        }

        private float GetMaxDensity()
        {
            float max = 0;
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    float density = simulation.Density[x, y];
                    if (density > max) max = density;
                }
            }
            return max;
        }
    }

    /// <summary>
    /// Represents a velocity vector at a specific position
    /// </summary>
    public struct VelocityVector
    {
        public float X, Y;      // Position
        public float U, V;      // Velocity components
        public float Magnitude; // Velocity magnitude
    }

    /// <summary>
    /// Simple example of how to use the fluid simulation
    /// </summary>
    public class FluidSimulationExample
    {
        public static void RunExample()
        {
            // Create simulation
            var simulation = new StableFluidSimulation(64, 64, 0.016f, 0.0001f, 0.0001f);
            var visualization = new FluidVisualization(simulation);

            // Add some initial forces and density
            simulation.AddForce(32, 32, 10.0f, 5.0f);
            simulation.AddDensity(32, 32, 100.0f);

            // Run simulation for several steps
            for (int i = 0; i < 100; i++)
            {
                // Add continuous force (like mouse movement)
                if (i % 10 == 0)
                {
                    simulation.AddForce(20 + i / 5, 32, 5.0f, 2.0f);
                    simulation.AddDensity(20 + i / 5, 32, 50.0f);
                }

                simulation.Step();
            }

            // Generate visualization
            var velocityBitmap = visualization.CreateVelocityMagnitudeBitmap();
            var densityBitmap = visualization.CreateDensityBitmap();
            var streamlines = visualization.GenerateStreamlines(10, 50, 0.3f);
            var vectors = visualization.GetVelocityVectors(4);

            // Save bitmaps (example)
            velocityBitmap.Save("velocity_field.png");
            densityBitmap.Save("density_field.png");

            Console.WriteLine($"Generated {streamlines.Count} streamlines");
            Console.WriteLine($"Generated {vectors.Count} velocity vectors");
        }
    }
}
