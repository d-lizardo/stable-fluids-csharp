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
            pManager.AddPointParameter("Force Points", "FP", "Points where forces are applied", GH_ParamAccess.list);
            pManager.AddVectorParameter("Force Vectors", "FV", "Force vectors to apply", GH_ParamAccess.list);
            pManager.AddPointParameter("Density Points", "DP", "Points where density is added", GH_ParamAccess.list);
            pManager.AddNumberParameter("Density Amount", "DA", "Amount of density to add", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Reset", "R", "Reset simulation", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Step", "S", "Perform simulation step", GH_ParamAccess.item, false);

            // Make some parameters optional
            pManager[4].Optional = true; // Force Points
            pManager[5].Optional = true; // Force Vectors
            pManager[6].Optional = true; // Density Points
            pManager[7].Optional = true; // Density Amount
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
            var forcePoints = new List<Point3d>();
            var forceVectors = new List<Vector3d>();
            var densityPoints = new List<Point3d>();
            var densityAmounts = new List<double>();
            bool reset = false, step = false;

            DA.GetData(0, ref width);
            DA.GetData(1, ref height);
            DA.GetData(2, ref viscosity);
            DA.GetData(3, ref diffusion);
            DA.GetDataList(4, forcePoints);
            DA.GetDataList(5, forceVectors);
            DA.GetDataList(6, densityPoints);
            DA.GetDataList(7, densityAmounts);
            DA.GetData(8, ref reset);
            DA.GetData(9, ref step);

            // Initialize or reset simulation
            if (!simulationInitialized || reset ||
                (simulation != null && (simulation.VelocityU.GetLength(0) != width ||
                simulation.VelocityU.GetLength(1) != height)))
            {
                simulation = new StableFluidSimulation(width, height, 0.016f, (float)viscosity, (float)diffusion);
                visualization = new FluidVisualization(simulation);
                simulationInitialized = true;
            }

            // Convert curve into steps
            //creat steps array
            //divide curve by time
            //get curve frames
            //append curve points
            //append curve tangents
            //set force = input

            // Get list of steps from input
            // for (step i = 0, i < len(steps), i++)
            // {
            //     //remove previous forces (optional)
            //     simulation.ResetForces(); //TODO implement resest of forces only
            //     //Apply forces
            //     simulation.AddForce(step.gridX, step.gridY, step.forceX, step.forceY); //Question: can this stack or is it unstable? Do I have to remove previous forces?
            //     //Step forward
            //     simulation.Step(timestep);
            // }
            
            // Apply forces
            if (forcePoints.Count > 0 && forceVectors.Count > 0)
            {
                int minCount = Math.Min(forcePoints.Count, forceVectors.Count);
                for (int i = 0; i < minCount; i++)
                {
                    // Convert world coordinates to grid coordinates
                    int gridX = (int)Math.Max(0, Math.Min(width - 1, forcePoints[i].X * width));
                    int gridY = (int)Math.Max(0, Math.Min(height - 1, forcePoints[i].Y * height));

                    // Scale force vectors appropriately
                    float forceX = (float)(forceVectors[i].X * 10.0);
                    float forceY = (float)(forceVectors[i].Y * 10.0);

                    simulation.AddForce(gridX, gridY, forceX, forceY);
                }
            }

            // Add density
            if (densityPoints.Count > 0 && densityAmounts.Count > 0)
            {
                int minCount = Math.Min(densityPoints.Count, densityAmounts.Count);
                for (int i = 0; i < minCount; i++)
                {
                    int gridX = (int)Math.Max(0, Math.Min(width - 1, densityPoints[i].X * width));
                    int gridY = (int)Math.Max(0, Math.Min(height - 1, densityPoints[i].Y * height));

                    simulation.AddDensity(gridX, gridY, (float)densityAmounts[i]);
                }
            }

            // Perform simulation step
            if (step)
            {
                simulation.Step();
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
            // var motionProfile = new MotionProfile(curvePoints, curveTangents);
            // DA.SetData(0, motionProfile);
        }
        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("d57e5ab1-2a29-4500-9781-f353aa591f0b"); }
        }
    }
}