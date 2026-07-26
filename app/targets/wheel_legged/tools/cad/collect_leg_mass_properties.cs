using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static class CollectLegMassProperties
{
    private const string LengthMateName = "WL_SCAN_LENGTH";
    private const string DirectionMateName = "WL_SCAN_DIRECTION";
    private const string PointWheelName = "点1";
    private const string PointUpperName = "点2";

    [DllImport("ole32.dll")]
    private static extern int CreateBindCtx(uint reserved, out IBindCtx bindContext);

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length != 4)
        {
            Console.Error.WriteLine(
                "Usage: collect_leg_mass_properties.exe <solidworks_pid> <working_assembly> <output_csv> <original_title>");
            return 2;
        }

        int pid = int.Parse(args[0], CultureInfo.InvariantCulture);
        string workingAssembly = Path.GetFullPath(args[1]);
        string outputCsv = Path.GetFullPath(args[2]);
        string originalTitle = args[3];

        ISldWorks sw = null;
        IModelDoc2 model = null;
        try
        {
            sw = GetSolidWorks(pid);
            if (sw == null)
                throw new InvalidOperationException("Cannot attach to SolidWorks PID " + pid);

            int openError = 0;
            int openWarning = 0;
            model = sw.OpenDoc6(
                workingAssembly,
                (int)swDocumentTypes_e.swDocASSEMBLY,
                (int)swOpenDocOptions_e.swOpenDocOptions_Silent,
                "",
                ref openError,
                ref openWarning);
            if (model == null)
                throw new InvalidOperationException(
                    string.Format("Cannot open working assembly; error={0}, warning={1}", openError, openWarning));

            int activateError = 0;
            sw.ActivateDoc3(
                model.GetTitle(),
                false,
                (int)swRebuildOnActivation_e.swDontRebuildActiveDoc,
                ref activateError);

            IAssemblyDoc assembly = (IAssemblyDoc)model;
            assembly.ResolveAllLightWeightComponents(true);

            IDimension lengthDimension = GetDimension(model, LengthMateName);
            IDimension directionDimension = GetDimension(model, DirectionMateName);
            double baseAngleRad = lengthDimension.SystemValue;
            double directionAngleRad = directionDimension.SystemValue;

            double[] initialUpper = GetPoint(model, PointUpperName);
            var rows = new List<string>();
            rows.Add(BuildHeader());

            double currentAngleRad = baseAngleRad;
            double currentLengthM = GetLegLength(model);
            double slopeMPerRad = 0.003714520 / (Math.PI / 180.0);

            rows.Add(CollectRow(
                model,
                "kinematic_min",
                currentLengthM,
                currentAngleRad,
                directionAngleRad,
                initialUpper,
                0.0,
                true));
            Console.WriteLine("kinematic_min: L={0:F9} m, angle={1:F6} deg",
                currentLengthM, Degrees(currentAngleRad));

            for (int i = 11; i <= 33; ++i)
            {
                double targetLengthM = i / 100.0;
                SolveTargetLength(
                    model,
                    lengthDimension,
                    targetLengthM,
                    baseAngleRad,
                    baseAngleRad + 67.0 * Math.PI / 180.0,
                    ref currentAngleRad,
                    ref currentLengthM,
                    ref slopeMPerRad);

                double residualM = currentLengthM - targetLengthM;
                bool valid = Math.Abs(residualM) <= 2.0e-6;
                rows.Add(CollectRow(
                    model,
                    "lookup_" + i.ToString(CultureInfo.InvariantCulture) + "cm",
                    targetLengthM,
                    currentAngleRad,
                    directionAngleRad,
                    initialUpper,
                    residualM,
                    valid));
                Console.WriteLine(
                    "lookup_{0}cm: L={1:F9} m, residual={2:E3} m, angle={3:F6} deg, valid={4}",
                    i, currentLengthM, residualM, Degrees(currentAngleRad), valid);
            }

            double maxAngleRad = baseAngleRad + 67.0 * Math.PI / 180.0;
            lengthDimension.SystemValue = maxAngleRad;
            bool maxRebuild = model.EditRebuild3();
            currentLengthM = GetLegLength(model);
            rows.Add(CollectRow(
                model,
                "kinematic_max",
                currentLengthM,
                maxAngleRad,
                directionAngleRad,
                initialUpper,
                0.0,
                maxRebuild));
            Console.WriteLine("kinematic_max: L={0:F9} m, angle={1:F6} deg, rebuild={2}",
                currentLengthM, Degrees(maxAngleRad), maxRebuild);

            Directory.CreateDirectory(Path.GetDirectoryName(outputCsv));
            File.WriteAllText(outputCsv, string.Join(System.Environment.NewLine, rows) + System.Environment.NewLine,
                new UTF8Encoding(true));
            Console.WriteLine("WROTE " + outputCsv);
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
            return 1;
        }
        finally
        {
            try
            {
                if (sw != null && model != null)
                {
                    string title = model.GetTitle();
                    int activateError = 0;
                    sw.ActivateDoc3(
                        originalTitle,
                        false,
                        (int)swRebuildOnActivation_e.swDontRebuildActiveDoc,
                        ref activateError);
                    sw.QuitDoc(title);
                }
            }
            catch (Exception closeError)
            {
                Console.Error.WriteLine("Failed to close working copy: " + closeError.Message);
            }
        }
    }

    private static void SolveTargetLength(
        IModelDoc2 model,
        IDimension dimension,
        double targetLengthM,
        double minAngleRad,
        double maxAngleRad,
        ref double currentAngleRad,
        ref double currentLengthM,
        ref double slopeMPerRad)
    {
        double previousAngle = currentAngleRad;
        double previousLength = currentLengthM;

        for (int iteration = 0; iteration < 6; ++iteration)
        {
            double errorM = targetLengthM - currentLengthM;
            if (Math.Abs(errorM) <= 1.0e-7)
                return;

            double safeSlope = Math.Abs(slopeMPerRad) > 1.0e-5 ? slopeMPerRad : 0.20;
            double nextAngle = currentAngleRad + errorM / safeSlope;
            nextAngle = Math.Max(minAngleRad, Math.Min(maxAngleRad, nextAngle));

            dimension.SystemValue = nextAngle;
            model.EditRebuild3();
            double nextLength = GetLegLength(model);

            double deltaAngle = nextAngle - currentAngleRad;
            if (Math.Abs(deltaAngle) > 1.0e-9)
            {
                double observedSlope = (nextLength - currentLengthM) / deltaAngle;
                if (observedSlope > 1.0e-5 && observedSlope < 1.0)
                    slopeMPerRad = observedSlope;
            }

            previousAngle = currentAngleRad;
            previousLength = currentLengthM;
            currentAngleRad = nextAngle;
            currentLengthM = nextLength;
        }

        // One secant correction is useful near the long-leg end where the slope changes rapidly.
        double denominator = currentLengthM - previousLength;
        if (Math.Abs(denominator) > 1.0e-9)
        {
            double secantAngle = currentAngleRad
                + (targetLengthM - currentLengthM) * (currentAngleRad - previousAngle) / denominator;
            secantAngle = Math.Max(minAngleRad, Math.Min(maxAngleRad, secantAngle));
            dimension.SystemValue = secantAngle;
            model.EditRebuild3();
            currentAngleRad = secantAngle;
            currentLengthM = GetLegLength(model);
        }
    }

    private static string BuildHeader()
    {
        var names = new List<string>
        {
            "sample_id", "requested_l0_m", "actual_length_angle_deg", "direction_angle_deg",
            "rebuild_ok", "length_residual_m",
            "point1_x_m", "point1_y_m", "point1_z_m",
            "point2_x_m", "point2_y_m", "point2_z_m", "point2_drift_m",
            "mass_kg", "volume_m3", "surface_area_m2",
            "com_x_m", "com_y_m", "com_z_m",
        };
        AddMatrixNames(names, "icom");
        AddMatrixNames(names, "iorigin");
        names.AddRange(new[]
        {
            "principal_i1_kg_m2", "principal_i2_kg_m2", "principal_i3_kg_m2",
            "principal_axis1_x", "principal_axis1_y", "principal_axis1_z",
            "principal_axis2_x", "principal_axis2_y", "principal_axis2_z",
            "principal_axis3_x", "principal_axis3_y", "principal_axis3_z",
        });
        return string.Join(",", names);
    }

    private static void AddMatrixNames(List<string> names, string prefix)
    {
        names.Add(prefix + "_xx_kg_m2");
        names.Add(prefix + "_xy_kg_m2");
        names.Add(prefix + "_xz_kg_m2");
        names.Add(prefix + "_yx_kg_m2");
        names.Add(prefix + "_yy_kg_m2");
        names.Add(prefix + "_yz_kg_m2");
        names.Add(prefix + "_zx_kg_m2");
        names.Add(prefix + "_zy_kg_m2");
        names.Add(prefix + "_zz_kg_m2");
    }

    private static string CollectRow(
        IModelDoc2 model,
        string sampleId,
        double requestedLengthM,
        double lengthAngleRad,
        double directionAngleRad,
        double[] initialUpper,
        double lengthResidualM,
        bool rebuildOk)
    {
        double[] point1 = GetPoint(model, PointWheelName);
        double[] point2 = GetPoint(model, PointUpperName);
        double point2Drift = Distance(point2, initialUpper);

        IMassProperty2 massProperty = (IMassProperty2)model.Extension.CreateMassProperty2();
        if (massProperty == null)
            throw new InvalidOperationException("CreateMassProperty2 returned null");
        massProperty.UseSystemUnits = true;
        massProperty.IncludeHiddenBodiesOrComponents = true;
        massProperty.AccuracyLevel = (int)swMassPropertyAccuracyLevel_e.swMassPropertyAccuracyLevel_Higher;
        massProperty.Recalculate();

        double[] centerOfMass = (double[])massProperty.CenterOfMass;
        double[] inertiaCom = (double[])massProperty.GetMomentOfInertia(
            (int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass);
        double[] inertiaOrigin = (double[])massProperty.GetMomentOfInertia(
            (int)swMassPropertyMoment_e.swMassPropertyMomentAboutCoordSys);
        double[] principalMoments = (double[])massProperty.PrincipalMomentsOfInertia;
        double[] principalAxis1 = (double[])massProperty.PrincipalAxesOfInertia[0];
        double[] principalAxis2 = (double[])massProperty.PrincipalAxesOfInertia[1];
        double[] principalAxis3 = (double[])massProperty.PrincipalAxesOfInertia[2];

        var values = new List<string>
        {
            CsvString(sampleId),
            Number(requestedLengthM),
            Number(Degrees(lengthAngleRad)),
            Number(Degrees(directionAngleRad)),
            rebuildOk ? "1" : "0",
            Number(lengthResidualM),
        };
        AddVector(values, point1);
        AddVector(values, point2);
        values.Add(Number(point2Drift));
        values.Add(Number(massProperty.Mass));
        values.Add(Number(massProperty.Volume));
        values.Add(Number(massProperty.SurfaceArea));
        AddVector(values, centerOfMass);
        AddVector(values, inertiaCom);
        AddVector(values, inertiaOrigin);
        AddVector(values, principalMoments);
        AddVector(values, principalAxis1);
        AddVector(values, principalAxis2);
        AddVector(values, principalAxis3);
        return string.Join(",", values);
    }

    private static ISldWorks GetSolidWorks(int processId)
    {
        IBindCtx context = null;
        IRunningObjectTable rot = null;
        IEnumMoniker monikers = null;
        try
        {
            CreateBindCtx(0, out context);
            context.GetRunningObjectTable(out rot);
            rot.EnumRunning(out monikers);
            var moniker = new IMoniker[1];
            string target = "SolidWorks_PID_" + processId.ToString(CultureInfo.InvariantCulture);
            while (monikers.Next(1, moniker, IntPtr.Zero) == 0)
            {
                string name = null;
                try
                {
                    moniker[0].GetDisplayName(context, null, out name);
                }
                catch
                {
                }
                if (!string.Equals(target, name, StringComparison.OrdinalIgnoreCase))
                    continue;
                object app;
                rot.GetObject(moniker[0], out app);
                return (ISldWorks)app;
            }
            return null;
        }
        finally
        {
            if (monikers != null) Marshal.ReleaseComObject(monikers);
            if (rot != null) Marshal.ReleaseComObject(rot);
            if (context != null) Marshal.ReleaseComObject(context);
        }
    }

    private static IDimension GetDimension(IModelDoc2 model, string featureName)
    {
        IFeature feature = FindFeature((IFeature)model.FirstFeature(), featureName);
        if (feature == null)
            throw new InvalidOperationException("Feature not found: " + featureName);
        IDisplayDimension displayDimension = (IDisplayDimension)feature.GetFirstDisplayDimension();
        if (displayDimension == null)
            throw new InvalidOperationException("Dimension not found on feature: " + featureName);
        return (IDimension)displayDimension.GetDimension2(0);
    }

    private static IFeature FindFeature(IFeature feature, string name)
    {
        while (feature != null)
        {
            if (feature.Name == name)
                return feature;

            IFeature child = (IFeature)feature.GetFirstSubFeature();
            while (child != null)
            {
                IFeature found = FindFeature(child, name);
                if (found != null)
                    return found;
                child = (IFeature)child.GetNextSubFeature();
            }
            feature = (IFeature)feature.GetNextFeature();
        }
        return null;
    }

    private static double[] GetPoint(IModelDoc2 model, string featureName)
    {
        IFeature feature = FindFeature((IFeature)model.FirstFeature(), featureName);
        if (feature == null)
            throw new InvalidOperationException("Reference point not found: " + featureName);
        IRefPoint point = (IRefPoint)feature.GetSpecificFeature2();
        return (double[])point.GetRefPoint().ArrayData;
    }

    private static double GetLegLength(IModelDoc2 model)
    {
        double[] point1 = GetPoint(model, PointWheelName);
        double[] point2 = GetPoint(model, PointUpperName);
        double dy = point1[1] - point2[1];
        double dz = point1[2] - point2[2];
        return Math.Sqrt(dy * dy + dz * dz);
    }

    private static double Distance(double[] a, double[] b)
    {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double Degrees(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    private static void AddVector(List<string> values, IEnumerable<double> vector)
    {
        values.AddRange(vector.Select(Number));
    }

    private static string Number(double value)
    {
        return value.ToString("R", CultureInfo.InvariantCulture);
    }

    private static string CsvString(string value)
    {
        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }
}
