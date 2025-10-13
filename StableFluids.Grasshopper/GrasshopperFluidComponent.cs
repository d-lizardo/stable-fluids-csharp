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
            pManager.AddNumberParameter("Viscosity", "Visc", "Fluid viscosity", GH_ParamAccess.item, 0.0001);
            pManager.AddNumberParameter("Diffusion", "Diff", "Density diffusion rate", GH_ParamAccess.item, 0.0001);
            pManager.AddGenericParameter("Motion Profile", "MP", "Motion profile defining the movement through fluid", GH_ParamAccess.item);
            pManager.AddNumberParameter("Progress", "P", "Progress through motion profile (0.0 to 1.0)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Force Scale", "FS", "Scale factor for applied forces", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Reset", "R", "Reset simulation", GH_ParamAccess.item, false);

            // Make motion profile optional for backward compatibility
            pManager[4].Optional = true; // Motion Profile
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Velocity Bitmap", "VB", "Bitmap showing velocity field", GH_ParamAccess.item);
            pManager.AddGenericParameter("Density Bitmap", "DB", "Bitmap showing density field", GH_ParamAccess.item);
            pManager.AddCurveParameter("Streamlines", "SL", "Streamlines showing flow", GH_ParamAccess.list);
            pManager.AddLineParameter("Velocity Vectors", "VV", "Velocity vectors", GH_ParamAccess.list);
            pManager.AddNumberParameter("Velocity Magnitudes", "VM", "Velocity magnitudes at grid points", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get input parameters
            int width = 64, height = 64;
            double viscosity = 0.0001, diffusion = 0.0001;
            FluidSimulation.MotionProfile motionProfile = null;
            double progress = 0.0;
            double forceScale = 10.0;
            bool reset = false;

            DA.GetData(0, ref width);
            DA.GetData(1, ref height);
            DA.GetData(2, ref viscosity);
            DA.GetData(3, ref diffusion);
            DA.GetData(4, ref motionProfile);
            DA.GetData(5, ref progress);
            DA.GetData(6, ref forceScale);
            DA.GetData(7, ref reset);

            // Initialize or reset simulation
            if (!simulationInitialized || reset ||
                (simulation != null && (simulation.VelocityU.GetLength(0) != width ||
                simulation.VelocityU.GetLength(1) != height)))
            {
                simulation = new StableFluidSimulation(width, height, 0.016f, (float)viscosity, (float)diffusion);
                visualization = new FluidVisualization(simulation);
                simulationInitialized = true;
            }

            // Execute motion profile if provided
            if (motionProfile != null && progress > 0)
            {
                // Reset simulation to clean state first
                simulation = new StableFluidSimulation(width, height, 0.016f, (float)viscosity, (float)diffusion);
                visualization = new FluidVisualization(simulation);

                // Execute motion profile up to the specified progress
                simulation.ExecuteMotionProfile(motionProfile, (float)progress, (float)forceScale);

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

            // Generate outputs
            var velocityBitmap = visualization.CreateVelocityMagnitudeBitmap();
            var densityBitmap = visualization.CreateDensityBitmap();
            var streamlines = GenerateStreamlineCurves();
            var velocityVectors = GenerateVelocityLines();
            var velocityMagnitudes = GetVelocityMagnitudes();

            // Set outputs
            DA.SetData(0, velocityBitmap);
            DA.SetData(1, densityBitmap);
            DA.SetDataList(2, streamlines);
            DA.SetDataList(3, velocityVectors);
            DA.SetDataList(4, velocityMagnitudes);
        }

        private List<Curve> GenerateStreamlineCurves()
        {
            var curves = new List<Curve>();
            var streamlines = visualization.GenerateStreamlines(15, 80, 0.4f);

            foreach (var streamline in streamlines)
            {
                if (streamline.Count < 2) continue;

                var points = new List<Point3d>();
                foreach (var pt in streamline)
                {
                    // Convert grid coordinates to world coordinates (0-1 range)
                    double x = pt.X / simulation.VelocityU.GetLength(0);
                    double y = pt.Y / simulation.VelocityU.GetLength(1);
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

        private List<Line> GenerateVelocityLines()
        {
            var lines = new List<Line>();
            var vectors = visualization.GetVelocityVectors(6);

            foreach (var vector in vectors)
            {
                // Convert to world coordinates
                double x = vector.X / (double)simulation.VelocityU.GetLength(0);
                double y = vector.Y / (double)simulation.VelocityU.GetLength(1);
                
                var start = new Point3d(x, y, 0);
                
                // Scale velocity for visualization
                double scale = 0.05; // Adjust this for better visualization
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
    /// Component to convert single curve to simulation steps
    /// </summary>
    public class MoveFromCurve : GH_Component
    {
        public MoveFromCurve()
            : base("Curve to Single Move", "MoveFromCurve",
                "Converts curve object to motion profile for fluid simu",
                "Physics", "Simulation")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "C", "Curve To Traverse", GH_ParamAccess.item);
            pManager.AddNumberParameter("Speed", "S", "Constant Speed of Traversal", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Timestep", "dT", "Simulation Timestep", GH_ParamAccess.item, 0.016f);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion Profile", "MP", "Motion Profile object defining all steps for simulation", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get input parameters
            Curve curve = null;
            double speed = 1.0;
            double timestep = 0.016;

            DA.GetData(0, ref curve);
            DA.GetData(1, ref speed);
            DA.GetData(2, ref timestep);

            // Calculate number of steps
            float stepDistance = (float)(speed * timestep);

            // Divide curve into points
            var curveParams = curve.DivideByLength(stepDistance, true);

            // Get points and tangents
            var curvePoints = new List<Point3d>();
            var curveTangents = new List<Vector3d>();
            for (int i = 0; i < curveParams.Length; i++)
            {
                curvePoints.Add(curve.PointAt(curveParams[i]));
                curveTangents.Add(curve.TangentAt(curveParams[i]));
            }

            // Create motion profile object
            var motionProfile = new FluidSimulation.MotionProfile(curvePoints, curveTangents, (float)timestep);
            DA.SetData(0, motionProfile);
        }
        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("d57e5ab1-2a29-4500-9781-f353aa591f0b"); }
        }
    }
}