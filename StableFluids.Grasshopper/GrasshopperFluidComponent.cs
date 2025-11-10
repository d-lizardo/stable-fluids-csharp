using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace FluidSimulation.Grasshopper
{
    /// <summary>
    /// Example Grasshopper component for fluid simulation
    /// This shows how to integrate the fluid simulation into Grasshopper
    /// </summary>
    public class FluidSimulationComponent : GH_Component
    {
        private StableFluidSimulation simulation;
        private FluidVisualization visualization;
        private bool simulationInitialized = false;

        public FluidSimulationComponent()
            : base("Fluid Simulation", "FluidSim",
                "Performs real-time fluid simulation using Jos Stam's Stable Fluids algorithm",
                "Physics", "Simulation")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Width", "W", "Simulation grid width", GH_ParamAccess.item, 64);
            pManager.AddIntegerParameter("Height", "H", "Simulation grid height", GH_ParamAccess.item, 64);
            pManager.AddNumberParameter("World Width", "WW", "Physical width of simulation domain", GH_ParamAccess.item, 100.0);
            pManager.AddNumberParameter("World Height", "WH", "Physical height of simulation domain", GH_ParamAccess.item, 100.0);
            pManager.AddNumberParameter("Viscosity", "Visc", "Fluid viscosity", GH_ParamAccess.item, 0.0001);
            pManager.AddNumberParameter("Diffusion", "Diff", "Density diffusion rate", GH_ParamAccess.item, 0.0001);
            pManager.AddIntegerParameter("Solver Iterations", "Iter", "Number of solver iterations (lower = faster, higher = more accurate)", GH_ParamAccess.item, 10);
            pManager.AddGenericParameter("Motion Profile", "MP", "Motion profile defining the movement through fluid", GH_ParamAccess.item);
            pManager.AddNumberParameter("Progress", "P", "Progress through motion profile (0.0 to 1.0)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Force Scale", "FS", "Scale factor for applied forces", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Reset", "R", "Reset simulation", GH_ParamAccess.item, false);

            // Make motion profile optional for backward compatibility
            pManager[7].Optional = true; // Motion Profile
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Velocity Field", "VF", "Colored mesh showing velocity field", GH_ParamAccess.item);
            pManager.AddMeshParameter("Density Field", "DF", "Colored mesh showing density field", GH_ParamAccess.item);
            pManager.AddCurveParameter("Streamlines", "SL", "Streamlines showing flow", GH_ParamAccess.list);
            pManager.AddLineParameter("Velocity Vectors", "VV", "Velocity vectors", GH_ParamAccess.list);
            pManager.AddNumberParameter("Velocity Magnitudes", "VM", "Velocity magnitudes at grid points", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get input parameters
            int width = 64, height = 64;
            double worldWidth = 100.0, worldHeight = 100.0;
            double viscosity = 0.0001, diffusion = 0.0001;
            int solverIterations = 10;
            FluidSimulation.MotionProfile motionProfile = null;
            double progress = 0.0;
            double forceScale = 10.0;
            bool reset = false;

            DA.GetData(0, ref width);
            DA.GetData(1, ref height);
            DA.GetData(2, ref worldWidth);
            DA.GetData(3, ref worldHeight);
            DA.GetData(4, ref viscosity);
            DA.GetData(5, ref diffusion);
            DA.GetData(6, ref solverIterations);
            DA.GetData(7, ref motionProfile);
            DA.GetData(8, ref progress);
            DA.GetData(9, ref forceScale);
            DA.GetData(10, ref reset);

            // Initialize or reset simulation
            if (!simulationInitialized || reset ||
                (simulation != null && (simulation.VelocityU.GetLength(0) != width ||
                simulation.VelocityU.GetLength(1) != height)))
            {
                simulation = new StableFluidSimulation(width, height, 0.016f, (float)viscosity, (float)diffusion, solverIterations);
                visualization = new FluidVisualization(simulation);
                simulationInitialized = true;
            }

            // Execute motion profile if provided
            if (motionProfile != null && progress > 0)
            {
                // Reset simulation to clean state first
                simulation = new StableFluidSimulation(width, height, 0.016f, (float)viscosity, (float)diffusion, solverIterations);
                visualization = new FluidVisualization(simulation);

                // Execute motion profile up to the specified progress with world scale
                simulation.ExecuteMotionProfile(motionProfile, (float)progress, (float)forceScale, worldWidth, worldHeight);

                // Debug output (you can remove this later)
                int stepsExecuted = (int)(motionProfile.StepCount * Math.Max(0, Math.Min(1, progress)));
                this.Message = $"Steps: {stepsExecuted}/{motionProfile.StepCount}";
            }
            else if (motionProfile == null)
            {
                this.Message = "No motion profile";
            }
            else
            {
                this.Message = $"Progress: {progress:F2}";
            }

            // Generate outputs with world scale
            var velocityMesh = CreateVelocityMesh(worldWidth, worldHeight);
            var densityMesh = CreateDensityMesh(worldWidth, worldHeight);
            var streamlines = GenerateStreamlineCurves(worldWidth, worldHeight);
            var velocityVectors = GenerateVelocityLines(worldWidth, worldHeight);
            var velocityMagnitudes = GetVelocityMagnitudes();

            // Set outputs
            DA.SetData(0, velocityMesh);
            DA.SetData(1, densityMesh);
            DA.SetDataList(2, streamlines);
            DA.SetDataList(3, velocityVectors);
            DA.SetDataList(4, velocityMagnitudes);
        }

        private Rhino.Geometry.Mesh CreateVelocityMesh(double worldWidth, double worldHeight)
        {
            var mesh = new Rhino.Geometry.Mesh();
            int gridWidth = simulation.VelocityU.GetLength(0);
            int gridHeight = simulation.VelocityU.GetLength(1);

            float maxMagnitude = GetMaxVelocityMagnitude();
            if (maxMagnitude == 0) maxMagnitude = 1;

            // Create vertices
            for (int y = 0; y < gridHeight; y++)
            {
                for (int x = 0; x < gridWidth; x++)
                {
                    double worldX = (x / (double)(gridWidth - 1)) * worldWidth;
                    double worldY = (y / (double)(gridHeight - 1)) * worldHeight;
                    mesh.Vertices.Add(worldX, worldY, 0);

                    // Add vertex color based on velocity magnitude
                    float magnitude = simulation.GetVelocityMagnitude(x, y);
                    float normalized = Math.Min(magnitude / maxMagnitude, 1.0f);
                    int intensity = (int)(normalized * 255);
                    mesh.VertexColors.Add(intensity, 0, 255 - intensity);
                }
            }

            // Create faces
            for (int y = 0; y < gridHeight - 1; y++)
            {
                for (int x = 0; x < gridWidth - 1; x++)
                {
                    int i = y * gridWidth + x;
                    mesh.Faces.AddFace(i, i + 1, i + gridWidth + 1, i + gridWidth);
                }
            }

            mesh.Normals.ComputeNormals();
            return mesh;
        }

        private Rhino.Geometry.Mesh CreateDensityMesh(double worldWidth, double worldHeight)
        {
            var mesh = new Rhino.Geometry.Mesh();
            int gridWidth = simulation.VelocityU.GetLength(0);
            int gridHeight = simulation.VelocityU.GetLength(1);

            float maxDensity = GetMaxDensity();
            if (maxDensity == 0) maxDensity = 1;

            // Create vertices
            for (int y = 0; y < gridHeight; y++)
            {
                for (int x = 0; x < gridWidth; x++)
                {
                    double worldX = (x / (double)(gridWidth - 1)) * worldWidth;
                    double worldY = (y / (double)(gridHeight - 1)) * worldHeight;
                    mesh.Vertices.Add(worldX, worldY, 0);

                    // Add vertex color based on density
                    float density = simulation.Density[x, y];
                    float normalized = Math.Min(density / maxDensity, 1.0f);
                    int intensity = (int)(normalized * 255);
                    mesh.VertexColors.Add(intensity, intensity, intensity);
                }
            }

            // Create faces
            for (int y = 0; y < gridHeight - 1; y++)
            {
                for (int x = 0; x < gridWidth - 1; x++)
                {
                    int i = y * gridWidth + x;
                    mesh.Faces.AddFace(i, i + 1, i + gridWidth + 1, i + gridWidth);
                }
            }

            mesh.Normals.ComputeNormals();
            return mesh;
        }

        private List<Curve> GenerateStreamlineCurves(double worldWidth, double worldHeight)
        {
            var curves = new List<Curve>();
            var streamlines = visualization.GenerateStreamlines(15, 80, 0.4f);
            int gridWidth = simulation.VelocityU.GetLength(0);
            int gridHeight = simulation.VelocityU.GetLength(1);

            foreach (var streamline in streamlines)
            {
                if (streamline.Count < 2) continue;

                var points = new List<Point3d>();
                foreach (var pt in streamline)
                {
                    // Convert grid coordinates to world coordinates
                    double x = (pt.X / gridWidth) * worldWidth;
                    double y = (pt.Y / gridHeight) * worldHeight;
                    points.Add(new Point3d(x, y, 0));
                }

                if (points.Count >= 2)
                {
                    var polyline = new Polyline(points);
                    curves.Add(polyline.ToNurbsCurve());
                }
            }

            return curves;
        }

        private List<Line> GenerateVelocityLines(double worldWidth, double worldHeight)
        {
            var lines = new List<Line>();
            var vectors = visualization.GetVelocityVectors(6);
            int gridWidth = simulation.VelocityU.GetLength(0);
            int gridHeight = simulation.VelocityU.GetLength(1);

            foreach (var vector in vectors)
            {
                // Convert to world coordinates
                double x = (vector.X / gridWidth) * worldWidth;
                double y = (vector.Y / gridHeight) * worldHeight;

                var start = new Point3d(x, y, 0);

                // Scale velocity for visualization (proportional to world size)
                double scale = Math.Min(worldWidth, worldHeight) * 0.02;
                var end = new Point3d(x + vector.U * scale, y + vector.V * scale, 0);

                lines.Add(new Line(start, end));
            }

            return lines;
        }

        private List<double> GetVelocityMagnitudes()
        {
            var magnitudes = new List<double>();
            int width = simulation.VelocityU.GetLength(0);
            int height = simulation.VelocityU.GetLength(1);

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    magnitudes.Add(simulation.GetVelocityMagnitude(x, y));
                }
            }

            return magnitudes;
        }

        private float GetMaxVelocityMagnitude()
        {
            float max = 0;
            int width = simulation.VelocityU.GetLength(0);
            int height = simulation.VelocityU.GetLength(1);

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
            int width = simulation.VelocityU.GetLength(0);
            int height = simulation.VelocityU.GetLength(1);

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

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can create a custom icon here
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("dd092c18-a225-42d4-8215-4497ed108ae0"); }
        }
    }

    /// <summary>
    /// Component to convert mouse/cursor position to fluid forces
    /// </summary>
    public class MouseToFluidForceComponent : GH_Component
    {
        private Point3d lastMousePos = Point3d.Unset;

        public MouseToFluidForceComponent()
            : base("Mouse to Fluid Force", "MouseFluid",
                "Converts mouse movement to fluid forces",
                "Physics", "Simulation")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Mouse Position", "MP", "Current mouse position (0-1 range)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Force Scale", "FS", "Scale factor for forces", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Force Point", "FP", "Point where force should be applied", GH_ParamAccess.item);
            pManager.AddVectorParameter("Force Vector", "FV", "Force vector based on mouse movement", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d mousePos = Point3d.Unset;
            double forceScale = 1.0;

            DA.GetData(0, ref mousePos);
            DA.GetData(1, ref forceScale);

            Vector3d forceVector = Vector3d.Zero;

            if (lastMousePos.IsValid && mousePos.IsValid)
            {
                // Calculate force based on mouse movement
                forceVector = (mousePos - lastMousePos) * forceScale;
            }

            lastMousePos = mousePos;

            DA.SetData(0, mousePos);
            DA.SetData(1, forceVector);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("0440b637-bc3f-49cb-9d4b-70c12fe97a5b"); }
        }
    }

    /// <summary>
    /// Component to convert multiple curves to simulation steps with linking motion
    /// </summary>
    public class MoveFromCurve : GH_Component
    {
        public MoveFromCurve()
            : base("Curves to Motion Profile", "CurvesToMotion",
                "Converts curves to motion profile with linking motion between curves",
                "Physics", "Simulation")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curves", "C", "Curves to traverse in sequence", GH_ParamAccess.list);
            pManager.AddNumberParameter("Speed", "S", "Constant speed of traversal", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Timestep", "dT", "Simulation timestep", GH_ParamAccess.item, 0.016f);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion Profile", "MP", "Motion profile object defining all steps for simulation", GH_ParamAccess.item);
            pManager.AddTextParameter("Info", "I", "Information about the motion profile", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get input parameters
            List<Curve> curves = new List<Curve>();
            double speed = 1.0;
            double timestep = 0.016;

            DA.GetDataList(0, curves);
            DA.GetData(1, ref speed);
            DA.GetData(2, ref timestep);

            if (curves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No curves provided");
                return;
            }

            // Create motion profile
            var motionProfile = new FluidSimulation.MotionProfile();
            float currentTime = 0f;
            float stepDistance = (float)(speed * timestep);

            int totalSteps = 0;
            int totalLinkSteps = 0;

            for (int curveIndex = 0; curveIndex < curves.Count; curveIndex++)
            {
                var curve = curves[curveIndex];

                // Divide curve into points
                var curveParams = curve.DivideByLength(stepDistance, true);

                // Add motion steps for this curve
                for (int i = 0; i < curveParams.Length; i++)
                {
                    Point3d position = curve.PointAt(curveParams[i]);
                    Vector3d tangent = curve.TangentAt(curveParams[i]);
                    tangent.Unitize();
                    Vector3d velocity = tangent * speed;

                    motionProfile.AddStep(position, velocity, currentTime);
                    currentTime += (float)timestep;
                    totalSteps++;
                }

                // Add linking motion to next curve (if there is one)
                if (curveIndex < curves.Count - 1)
                {
                    Point3d endOfCurrentCurve = curve.PointAtEnd;
                    Point3d startOfNextCurve = curves[curveIndex + 1].PointAtStart;

                    double linkDistance = endOfCurrentCurve.DistanceTo(startOfNextCurve);
                    int linkSteps = (int)Math.Ceiling(linkDistance / stepDistance);

                    // Add steps with zero velocity (no forces applied, just time passing)
                    for (int i = 0; i < linkSteps; i++)
                    {
                        // Interpolate position during link (even though no forces are applied)
                        double t = (i + 1) / (double)linkSteps;
                        Point3d linkPosition = endOfCurrentCurve + t * (startOfNextCurve - endOfCurrentCurve);

                        motionProfile.AddStep(linkPosition, Vector3d.Zero, currentTime);
                        currentTime += (float)timestep;
                        totalLinkSteps++;
                    }
                }
            }

            // Output motion profile and info
            DA.SetData(0, motionProfile);
            DA.SetData(1, $"Curves: {curves.Count} | Motion Steps: {totalSteps} | Link Steps: {totalLinkSteps} | Total: {totalSteps + totalLinkSteps}");
        }
        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("d57e5ab1-2a29-4500-9781-f353aa591f0b"); }
        }
    }
}