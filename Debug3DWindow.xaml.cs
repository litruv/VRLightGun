using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace VRLightGun;

/// <summary>
/// Data for a single controller in the debug view.
/// </summary>
public class ControllerDebugData
{
    public string Name { get; set; } = "";
    public Point3D Position { get; set; }
    public Vector3D Direction { get; set; }
    public float XNorm { get; set; }
    public float YNorm { get; set; }
    public System.Windows.Media.Color Color { get; set; } = Colors.LimeGreen;
}

/// <summary>
/// 3D debug visualization window for calibration and pointing.
/// </summary>
public partial class Debug3DWindow : Window
{
    private Point3D[] _calibrationPoints = new Point3D[3];
    private readonly List<ControllerDebugData> _controllers = new();
    private double _cameraDistance = 3.0;

    public Debug3DWindow()
    {
        InitializeComponent();
        CreateGroundPlane();
    }

    private void CreateGroundPlane()
    {
        var group = new Model3DGroup();

        // Create grid lines
        for (int i = -5; i <= 5; i++)
        {
            // X-axis lines
            group.Children.Add(CreateLine(
                new Point3D(i * 0.5, 0, -2.5),
                new Point3D(i * 0.5, 0, 2.5),
                System.Windows.Media.Color.FromRgb(60, 60, 60)));

            // Z-axis lines
            group.Children.Add(CreateLine(
                new Point3D(-2.5, 0, i * 0.5),
                new Point3D(2.5, 0, i * 0.5),
                System.Windows.Media.Color.FromRgb(60, 60, 60)));
        }

        // Axis indicators
        group.Children.Add(CreateLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0), Colors.Red));    // X = Red
        group.Children.Add(CreateLine(new Point3D(0, 0, 0), new Point3D(0, 1, 0), Colors.Green));  // Y = Green
        group.Children.Add(CreateLine(new Point3D(0, 0, 0), new Point3D(0, 0, 1), Colors.Blue));   // Z = Blue

        GroundPlane.Content = group;
    }

    /// <summary>
    /// Updates the debug visualization with multiple controllers.
    /// </summary>
    /// <param name="calibrationPoints">The three calibration points (TL, TR, BR).</param>
    /// <param name="controllers">List of controller data to visualize.</param>
    public void UpdateVisualization(Point3D[] calibrationPoints, List<ControllerDebugData> controllers)
    {
        _calibrationPoints = calibrationPoints;
        _controllers.Clear();
        _controllers.AddRange(controllers);

        Dispatcher.Invoke(() =>
        {
            UpdateCalibrationPoints();
            UpdateScreenPlane();
            UpdatePointingRays();
            UpdateControllers();
            UpdateInfoText();
        });
    }

    /// <summary>
    /// Updates the debug visualization with current data (legacy single-controller support).
    /// </summary>
    public void UpdateVisualization(
        Point3D[] calibrationPoints,
        Vector3D pointingDirection,
        Point3D controllerPosition,
        float xNorm,
        float yNorm)
    {
        var controllers = new List<ControllerDebugData>
        {
            new ControllerDebugData
            {
                Name = "Controller",
                Position = controllerPosition,
                Direction = pointingDirection,
                XNorm = xNorm,
                YNorm = yNorm,
                Color = Colors.LimeGreen
            }
        };
        UpdateVisualization(calibrationPoints, controllers);
    }

    private void UpdateCalibrationPoints()
    {
        var group = new Model3DGroup();

        string[] labels = { "TL", "TR", "BR" };
        System.Windows.Media.Color[] colors = { Colors.Yellow, Colors.Cyan, Colors.Magenta };

        for (int i = 0; i < 3; i++)
        {
            if (_calibrationPoints[i] != default)
            {
                // Draw sphere at calibration position (where controller was)
                group.Children.Add(CreateSphere(_calibrationPoints[i], 0.05, colors[i]));
            }
        }

        CalibrationPointsVisual.Content = group;
    }

    private void UpdateScreenPlane()
    {
        if (_calibrationPoints[0] == default || _calibrationPoints[1] == default || _calibrationPoints[2] == default)
            return;

        var group = new Model3DGroup();

        // Draw calibration triangle/quad outline
        group.Children.Add(CreateLine(_calibrationPoints[0], _calibrationPoints[1], Colors.White)); // TL -> TR
        group.Children.Add(CreateLine(_calibrationPoints[1], _calibrationPoints[2], Colors.White)); // TR -> BR

        // Extrapolate BL
        var bl = new Point3D(
            _calibrationPoints[0].X + (_calibrationPoints[2].X - _calibrationPoints[1].X),
            _calibrationPoints[0].Y + (_calibrationPoints[2].Y - _calibrationPoints[1].Y),
            _calibrationPoints[0].Z + (_calibrationPoints[2].Z - _calibrationPoints[1].Z));

        group.Children.Add(CreateLine(_calibrationPoints[2], bl, Colors.Gray)); // BR -> BL
        group.Children.Add(CreateLine(bl, _calibrationPoints[0], Colors.Gray)); // BL -> TL

        ScreenPlaneVisual.Content = group;
    }

    private void UpdatePointingRays()
    {
        var group = new Model3DGroup();

        foreach (var controller in _controllers)
        {
            if (controller.Direction != default)
            {
                var rayEnd = controller.Position + controller.Direction * 2.0;
                group.Children.Add(CreateLine(controller.Position, rayEnd, controller.Color));
                group.Children.Add(CreateSphere(rayEnd, 0.02, controller.Color));
            }
        }

        PointingRayVisual.Content = group;
    }

    private void UpdateControllers()
    {
        var group = new Model3DGroup();

        foreach (var controller in _controllers)
        {
            if (controller.Position != default)
            {
                group.Children.Add(CreateSphere(controller.Position, 0.05, controller.Color));
            }
        }

        ControllerVisual.Content = group;
    }

    private void UpdateInfoText()
    {
        var lines = new List<string>();
        foreach (var controller in _controllers)
        {
            lines.Add($"{controller.Name}: X={controller.XNorm:F3} Y={controller.YNorm:F3}");
            lines.Add($"  Dir: ({controller.Direction.X:F2}, {controller.Direction.Y:F2}, {controller.Direction.Z:F2})");
            lines.Add($"  Pos: ({controller.Position.X:F2}, {controller.Position.Y:F2}, {controller.Position.Z:F2})");
        }
        InfoText.Text = string.Join("\n", lines);
    }

    private void CameraAngleSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (Camera == null) return;

        double angle = e.NewValue * Math.PI / 180.0;
        double x = Math.Sin(angle) * _cameraDistance;
        double z = Math.Cos(angle) * _cameraDistance;

        Camera.Position = new Point3D(x, 1.5, z);
        Camera.LookDirection = new Vector3D(-x, -0.3, -z);
    }

    private static GeometryModel3D CreateLine(Point3D start, Point3D end, System.Windows.Media.Color color)
    {
        var direction = end - start;
        var length = direction.Length;
        direction.Normalize();

        // Create a thin box as a line
        const double thickness = 0.005;
        var mesh = new MeshGeometry3D();

        // Calculate perpendicular vectors
        var up = Math.Abs(direction.Y) < 0.99 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0);
        var right = Vector3D.CrossProduct(direction, up);
        right.Normalize();
        up = Vector3D.CrossProduct(right, direction);
        up.Normalize();

        right *= thickness;
        up *= thickness;

        // Create box vertices along the line
        var p0 = start - right - up;
        var p1 = start + right - up;
        var p2 = start + right + up;
        var p3 = start - right + up;
        var p4 = end - right - up;
        var p5 = end + right - up;
        var p6 = end + right + up;
        var p7 = end - right + up;

        mesh.Positions.Add(p0); mesh.Positions.Add(p1); mesh.Positions.Add(p2); mesh.Positions.Add(p3);
        mesh.Positions.Add(p4); mesh.Positions.Add(p5); mesh.Positions.Add(p6); mesh.Positions.Add(p7);

        // Front face
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(2);
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(3);
        // Back face
        mesh.TriangleIndices.Add(4); mesh.TriangleIndices.Add(6); mesh.TriangleIndices.Add(5);
        mesh.TriangleIndices.Add(4); mesh.TriangleIndices.Add(7); mesh.TriangleIndices.Add(6);
        // Top face
        mesh.TriangleIndices.Add(3); mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(6);
        mesh.TriangleIndices.Add(3); mesh.TriangleIndices.Add(6); mesh.TriangleIndices.Add(7);
        // Bottom face
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(5); mesh.TriangleIndices.Add(1);
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(4); mesh.TriangleIndices.Add(5);
        // Left face
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(3); mesh.TriangleIndices.Add(7);
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(7); mesh.TriangleIndices.Add(4);
        // Right face
        mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(5); mesh.TriangleIndices.Add(6);
        mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(6); mesh.TriangleIndices.Add(2);

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        return new GeometryModel3D(mesh, material);
    }

    private static GeometryModel3D CreateSphere(Point3D center, double radius, System.Windows.Media.Color color)
    {
        var mesh = new MeshGeometry3D();
        const int segments = 12;
        const int rings = 8;

        // Generate sphere vertices
        for (int ring = 0; ring <= rings; ring++)
        {
            double phi = Math.PI * ring / rings;
            double y = Math.Cos(phi) * radius;
            double ringRadius = Math.Sin(phi) * radius;

            for (int seg = 0; seg <= segments; seg++)
            {
                double theta = 2 * Math.PI * seg / segments;
                double x = Math.Cos(theta) * ringRadius;
                double z = Math.Sin(theta) * ringRadius;

                mesh.Positions.Add(new Point3D(center.X + x, center.Y + y, center.Z + z));
            }
        }

        // Generate triangles
        for (int ring = 0; ring < rings; ring++)
        {
            for (int seg = 0; seg < segments; seg++)
            {
                int current = ring * (segments + 1) + seg;
                int next = current + segments + 1;

                mesh.TriangleIndices.Add(current);
                mesh.TriangleIndices.Add(next);
                mesh.TriangleIndices.Add(current + 1);

                mesh.TriangleIndices.Add(current + 1);
                mesh.TriangleIndices.Add(next);
                mesh.TriangleIndices.Add(next + 1);
            }
        }

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        return new GeometryModel3D(mesh, material);
    }
}
