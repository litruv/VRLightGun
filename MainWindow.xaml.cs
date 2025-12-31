using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace VRLightGun;

/// <summary>
/// Interaction logic for MainWindow.xaml
/// </summary>
public partial class MainWindow : Window
{
    private enum CalibrationStep
    {
        None,
        TopLeft,
        TopRight,
        BottomRight,
        Complete
    }

    private CalibrationStep _currentStep = CalibrationStep.None;
    private readonly Point3D[] _calibrationDirections = new Point3D[3];
    private readonly Point3D[] _calibrationPositions = new Point3D[3];

    // SteamVR/OpenVR fields
    private Valve.VR.CVRSystem? _vrSystem;
    private System.Threading.CancellationTokenSource? _pollingCts;

    // ViGEm fields
    private Nefarius.ViGEm.Client.ViGEmClient? _vigemClient;
    private Nefarius.ViGEm.Client.Targets.IXbox360Controller? _virtualGun;

    // 3D Debug window
    private Debug3DWindow? _debug3DWindow;

    public MainWindow()
    {
        InitializeComponent();
        this.StateChanged += MainWindow_StateChanged;
        this.Closing += MainWindow_Closing;
        OpenDebug3DWindow();
        LoadCalibration();
    }

    private void OpenDebug3DWindow()
    {
        _debug3DWindow = new Debug3DWindow();
        _debug3DWindow.Show();
    }

    private void MainWindow_StateChanged(object? sender, EventArgs e)
    {
        if (this.WindowState == WindowState.Minimized)
        {
            this.Hide();
            this.ShowInTaskbar = false;
        }
    }

    private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
    {
        // Hide instead of close, unless app is shutting down
        if (System.Windows.Application.Current?.ShutdownMode != ShutdownMode.OnExplicitShutdown)
        {
            e.Cancel = true;
            this.Hide();
            this.ShowInTaskbar = false;
        }
        _pollingCts?.Cancel();
        _vrSystem = null;
    }

    private async void StartCalibrationButton_Click(object sender, RoutedEventArgs e)
    {
        if (!InitOpenVR())
        {
            CalibrationInstructions.Text = "Failed to initialize SteamVR/OpenVR.";
            return;
        }
        _currentStep = CalibrationStep.TopLeft;
        CalibrationInstructions.Text = "Point the controller at the TOP-LEFT corner of your screen and press the trigger.";
        StartCalibrationButton.IsEnabled = false;
        _pollingCts = new System.Threading.CancellationTokenSource();
        await PollControllerForCalibration(_pollingCts.Token);

        // After calibration, start ViGEm emulation
        if (_currentStep == CalibrationStep.Complete)
        {
            StartViGEmEmulation();
        }
    }

    private bool InitOpenVR()
    {
        if (_vrSystem != null)
            return true;
        var error = Valve.VR.EVRInitError.None;
        _vrSystem = Valve.VR.OpenVR.Init(ref error, Valve.VR.EVRApplicationType.VRApplication_Overlay);
        return error == Valve.VR.EVRInitError.None && _vrSystem != null;
    }

    private async System.Threading.Tasks.Task PollControllerForCalibration(System.Threading.CancellationToken token)
    {
        // Find the right-hand controller (can be improved for user selection)
        uint rightHand = _vrSystem!.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.RightHand);
        if (rightHand == Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            CalibrationInstructions.Text = "No right-hand controller detected.";
            return;
        }

        // Trigger button mask: k_EButton_SteamVR_Trigger = 33
        const ulong triggerMask = 1UL << 33;

        while (_currentStep != CalibrationStep.Complete && !token.IsCancellationRequested)
        {
            Valve.VR.VRControllerState_t state = new Valve.VR.VRControllerState_t();
            Valve.VR.TrackedDevicePose_t[] poses = new Valve.VR.TrackedDevicePose_t[Valve.VR.OpenVR.k_unMaxTrackedDeviceCount];
            _vrSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseStanding, 0, poses);

            bool triggerDown = false;
            if (_vrSystem.GetControllerState(rightHand, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRControllerState_t))))
            {
                triggerDown = (state.ulButtonPressed & triggerMask) != 0;
            }

            // Get pose for debug visualization even before trigger press
            var pose = poses[rightHand];
            if (pose.bPoseIsValid)
            {
                var m = pose.mDeviceToAbsoluteTracking;
                var controllerPos = new Point3D(m.m3, m.m7, m.m11);
                var pointingDir = GetPointingDirection(m);

                // Update debug visualization during calibration
                Dispatcher.Invoke(() => UpdateDebugDuringCalibration(controllerPos, pointingDir));
            }

            // Wait for trigger press
            if (triggerDown)
            {
                // Get pose
                if (pose.bPoseIsValid)
                {
                    var m = pose.mDeviceToAbsoluteTracking;
                    var controllerPos = new Point3D(m.m3, m.m7, m.m11);
                    
                    // Get the pointing direction with -45° pitch applied
                    var pointingDir = GetPointingDirection(m);
                    RecordCalibrationPoint(pointingDir, controllerPos);

                    // Wait for trigger release before next point
                    while (triggerDown && !token.IsCancellationRequested)
                    {
                        if (_vrSystem.GetControllerState(rightHand, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRControllerState_t))))
                        {
                            triggerDown = (state.ulButtonPressed & triggerMask) != 0;
                        }
                        await System.Threading.Tasks.Task.Delay(10, token);
                    }
                }
            }
            await System.Threading.Tasks.Task.Delay(10, token);
        }
        if (_currentStep == CalibrationStep.Complete)
        {
            CalibrationInstructions.Text = "Calibration complete!";
            StartCalibrationButton.IsEnabled = true;
        }
    }

    private void RecordCalibrationPoint(Point3D direction, Point3D position)
    {
        switch (_currentStep)
        {
            case CalibrationStep.TopLeft:
                _calibrationDirections[0] = direction;
                _calibrationPositions[0] = position;
                CalibrationInstructions.Text = "Now point at the TOP-RIGHT corner and press the trigger.";
                _currentStep = CalibrationStep.TopRight;
                break;
            case CalibrationStep.TopRight:
                _calibrationDirections[1] = direction;
                _calibrationPositions[1] = position;
                CalibrationInstructions.Text = "Now point at the BOTTOM-RIGHT corner and press the trigger.";
                _currentStep = CalibrationStep.BottomRight;
                break;
            case CalibrationStep.BottomRight:
                _calibrationDirections[2] = direction;
                _calibrationPositions[2] = position;
                _currentStep = CalibrationStep.Complete;
                SaveCalibration();
                break;
        }
    }

    private void UpdateDebugDuringCalibration(Point3D controllerPos, Point3D pointingDir)
    {
        if (_debug3DWindow == null) return;

        var pos3D = new System.Windows.Media.Media3D.Point3D(controllerPos.X, controllerPos.Y, controllerPos.Z);
        var dir3D = new System.Windows.Media.Media3D.Vector3D(pointingDir.X, pointingDir.Y, pointingDir.Z);

        // Convert recorded calibration positions for visualization
        var calibPositions = new System.Windows.Media.Media3D.Point3D[3];
        for (int i = 0; i < 3; i++)
        {
            if (_calibrationPositions[i].X != 0 || _calibrationPositions[i].Y != 0 || _calibrationPositions[i].Z != 0)
            {
                calibPositions[i] = new System.Windows.Media.Media3D.Point3D(
                    _calibrationPositions[i].X,
                    _calibrationPositions[i].Y,
                    _calibrationPositions[i].Z);
            }
        }

        _debug3DWindow.UpdateVisualization(calibPositions, dir3D, pos3D, 0.5f, 0.5f);
    }

    /// <summary>
    /// Starts ViGEm emulation of a virtual Xbox 360 controller as a light gun.
    /// </summary>
    private void StartViGEmEmulation()
    {
        if (_vigemClient == null)
        {
            _vigemClient = new Nefarius.ViGEm.Client.ViGEmClient();
        }
        if (_virtualGun == null)
        {
            _virtualGun = _vigemClient.CreateXbox360Controller();
            _virtualGun.Connect();
        }

        // Start polling controller pose and trigger to update the virtual device
        System.Threading.Tasks.Task.Run(async () =>
        {
            while (true)
            {
                if (_vrSystem == null)
                    break;

                uint rightHand = _vrSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.RightHand);
                if (rightHand == Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
                    break;

                Valve.VR.VRControllerState_t state = new Valve.VR.VRControllerState_t();
                Valve.VR.TrackedDevicePose_t[] poses = new Valve.VR.TrackedDevicePose_t[Valve.VR.OpenVR.k_unMaxTrackedDeviceCount];
                _vrSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseStanding, 0, poses);

                // Get all button states
                bool triggerPressed = false;
                bool gripPressed = false;
                bool menuPressed = false;
                bool trackpadPressed = false;
                bool trackpadTouched = false;
                bool systemPressed = false;
                if (_vrSystem.GetControllerState(rightHand, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRControllerState_t))))
                {
                    const ulong triggerMask = 1UL << 33;
                    const ulong gripMask = 1UL << 2; // k_EButton_Grip = 2
                    const ulong menuMask = 1UL << 1; // k_EButton_ApplicationMenu = 1
                    const ulong trackpadMask = 1UL << 32; // k_EButton_SteamVR_Touchpad = 32
                    const ulong systemMask = 1UL << 0; // k_EButton_System = 0

                    triggerPressed = (state.ulButtonPressed & triggerMask) != 0;
                    gripPressed = (state.ulButtonPressed & gripMask) != 0;
                    menuPressed = (state.ulButtonPressed & menuMask) != 0;
                    trackpadPressed = (state.ulButtonPressed & trackpadMask) != 0;
                    systemPressed = (state.ulButtonPressed & systemMask) != 0;
                    trackpadTouched = (state.ulButtonTouched & trackpadMask) != 0;
                }

                var pose = poses[rightHand];
                if (pose.bPoseIsValid)
                {
                    var m = pose.mDeviceToAbsoluteTracking;

                    // Get current pointing direction with -45° pitch
                    var dir = GetPointingDirection(m);
                    var controllerPos = new Point3D(m.m3, m.m7, m.m11);

                    // Use ray-plane intersection to find screen coordinates
                    // Calibration positions define the screen plane: TL, TR, BR
                    var TL = _calibrationPositions[0];
                    var TR = _calibrationPositions[1];
                    var BR = _calibrationPositions[2];

                    // Calculate screen axes from calibration positions
                    // X axis: TL -> TR
                    double screenXx = TR.X - TL.X;
                    double screenXy = TR.Y - TL.Y;
                    double screenXz = TR.Z - TL.Z;
                    
                    // Y axis: TL -> BL (extrapolate BL = TL + (BR - TR))
                    double blX = TL.X + (BR.X - TR.X);
                    double blY = TL.Y + (BR.Y - TR.Y);
                    double blZ = TL.Z + (BR.Z - TR.Z);
                    double screenYx = blX - TL.X;
                    double screenYy = blY - TL.Y;
                    double screenYz = blZ - TL.Z;

                    // Screen normal (cross product of X and Y axes)
                    double normalX = screenXy * screenYz - screenXz * screenYy;
                    double normalY = screenXz * screenYx - screenXx * screenYz;
                    double normalZ = screenXx * screenYy - screenXy * screenYx;

                    // Ray-plane intersection: find t where (controllerPos + t * dir) hits the plane
                    // Plane equation: normal · (P - TL) = 0
                    // Substitute ray: normal · (controllerPos + t * dir - TL) = 0
                    // Solve for t: t = normal · (TL - controllerPos) / (normal · dir)
                    double denom = normalX * dir.X + normalY * dir.Y + normalZ * dir.Z;
                    
                    float xNorm = 0.5f, yNorm = 0.5f;
                    if (Math.Abs(denom) > 1e-9)
                    {
                        double toPlaneX = TL.X - controllerPos.X;
                        double toPlaneY = TL.Y - controllerPos.Y;
                        double toPlaneZ = TL.Z - controllerPos.Z;
                        double t = (normalX * toPlaneX + normalY * toPlaneY + normalZ * toPlaneZ) / denom;

                        // Only use if ray goes forward (t > 0)
                        if (t > 0)
                        {
                            // Hit point on the plane
                            double hitX = controllerPos.X + t * dir.X;
                            double hitY = controllerPos.Y + t * dir.Y;
                            double hitZ = controllerPos.Z + t * dir.Z;

                            // Convert hit point to screen UV coordinates
                            // Offset from TL
                            double offsetX = hitX - TL.X;
                            double offsetY = hitY - TL.Y;
                            double offsetZ = hitZ - TL.Z;

                            // Project onto screen axes
                            double screenXlen2 = screenXx * screenXx + screenXy * screenXy + screenXz * screenXz;
                            double screenYlen2 = screenYx * screenYx + screenYy * screenYy + screenYz * screenYz;

                            if (screenXlen2 > 1e-9 && screenYlen2 > 1e-9)
                            {
                                double dotX = offsetX * screenXx + offsetY * screenXy + offsetZ * screenXz;
                                double dotY = offsetX * screenYx + offsetY * screenYy + offsetZ * screenYz;
                                xNorm = (float)(dotX / screenXlen2);
                                yNorm = (float)(dotY / screenYlen2);
                            }
                        }
                    }

                    // Clamp to [0,1]
                    xNorm = Math.Clamp(xNorm, 0f, 1f);
                    yNorm = Math.Clamp(yNorm, 0f, 1f);

                    // Map 0-1 to full stick range (-32768 to 32767)
                    // Invert Y so top of screen = top of stick
                    short stickX = (short)(xNorm * 65535 - 32768);
                    short stickY = (short)((1f - yNorm) * 65535 - 32768);

                    // Update debug text boxes on UI thread
                    Dispatcher.Invoke(() =>
                    {
                        DebugXBox.Text = xNorm.ToString("F3");
                        DebugYBox.Text = yNorm.ToString("F3");

                        // Update 3D debug window
                        if (_debug3DWindow != null)
                        {
                            // Get controller position for visualization
                            var controllerPos = new System.Windows.Media.Media3D.Point3D(m.m3, m.m7, m.m11);
                            var pointingDir = new System.Windows.Media.Media3D.Vector3D(dir.X, dir.Y, dir.Z);
                            // Use calibration positions (where controller was), not directions
                            var calibPositions = new System.Windows.Media.Media3D.Point3D[]
                            {
                                new System.Windows.Media.Media3D.Point3D(_calibrationPositions[0].X, _calibrationPositions[0].Y, _calibrationPositions[0].Z),
                                new System.Windows.Media.Media3D.Point3D(_calibrationPositions[1].X, _calibrationPositions[1].Y, _calibrationPositions[1].Z),
                                new System.Windows.Media.Media3D.Point3D(_calibrationPositions[2].X, _calibrationPositions[2].Y, _calibrationPositions[2].Z)
                            };
                            _debug3DWindow.UpdateVisualization(calibPositions, pointingDir, controllerPos, xNorm, yNorm);
                        }
                    });

                    // Apply absolute position to left stick
                    _virtualGun.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbX, stickX);
                    _virtualGun.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbY, stickY);

                    // Map VR buttons to Xbox 360 buttons
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.A, triggerPressed); // Trigger
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.B, gripPressed); // Grip
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.X, menuPressed); // Menu
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Y, trackpadPressed); // Touchpad press
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Start, systemPressed); // System
                    _virtualGun.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Back, trackpadTouched); // Touchpad touch

                    _virtualGun.SubmitReport();
                }

                await System.Threading.Tasks.Task.Delay(10);
            }
        });
    }

    /// <summary>
    /// Gets the pointing direction from a controller pose matrix with 45° pitch applied.
    /// </summary>
    private Point3D GetPointingDirection(Valve.VR.HmdMatrix34_t m)
    {
        // Controller's local axes from the 3x4 matrix (column vectors):
        // Right (X): m.m0, m.m4, m.m8
        // Up (Y):    m.m1, m.m5, m.m9
        // Forward (-Z): -m.m2, -m.m6, -m.m10 (negative Z is forward in OpenVR)
        double upX = m.m1, upY = m.m5, upZ = m.m9;
        double fwdX = -m.m2, fwdY = -m.m6, fwdZ = -m.m10;

        // Apply 45 degree pitch: rotate forward vector towards down (negative up)
        // This is a simple rotation in the plane defined by forward and up vectors
        double pitchRad = 45.0 * Math.PI / 180.0;
        double cosP = Math.Cos(pitchRad);
        double sinP = Math.Sin(pitchRad);

        // New direction = forward * cos(pitch) - up * sin(pitch)
        // (negative pitch means rotating towards -up, i.e., down)
        double dx = fwdX * cosP - upX * sinP;
        double dy = fwdY * cosP - upY * sinP;
        double dz = fwdZ * cosP - upZ * sinP;

        // Normalize
        double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 1e-9)
        {
            dx /= len;
            dy /= len;
            dz /= len;
        }

        return new Point3D(dx, dy, dz);
    }

    private static string GetCalibrationFilePath()
    {
        var exePath = System.Reflection.Assembly.GetExecutingAssembly().Location;
        var exeDir = System.IO.Path.GetDirectoryName(exePath) ?? ".";
        return System.IO.Path.Combine(exeDir, "calibration.json");
    }

    private void SaveCalibration()
    {
        try
        {
            var data = new CalibrationData
            {
                Directions = _calibrationDirections.Select(p => new double[] { p.X, p.Y, p.Z }).ToArray(),
                Positions = _calibrationPositions.Select(p => new double[] { p.X, p.Y, p.Z }).ToArray()
            };
            var json = JsonSerializer.Serialize(data, new JsonSerializerOptions { WriteIndented = true });
            System.IO.File.WriteAllText(GetCalibrationFilePath(), json);
        }
        catch (Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Failed to save calibration: {ex.Message}");
        }
    }

    private void LoadCalibration()
    {
        try
        {
            var path = GetCalibrationFilePath();
            if (!System.IO.File.Exists(path))
                return;

            var json = System.IO.File.ReadAllText(path);
            var data = JsonSerializer.Deserialize<CalibrationData>(json);
            if (data?.Directions != null && data.Positions != null && data.Directions.Length == 3 && data.Positions.Length == 3)
            {
                for (int i = 0; i < 3; i++)
                {
                    _calibrationDirections[i] = new Point3D(data.Directions[i][0], data.Directions[i][1], data.Directions[i][2]);
                    _calibrationPositions[i] = new Point3D(data.Positions[i][0], data.Positions[i][1], data.Positions[i][2]);
                }
                _currentStep = CalibrationStep.Complete;
                CalibrationInstructions.Text = "Calibration loaded from file.";

                // Auto-start emulation with loaded calibration
                if (InitOpenVR())
                {
                    StartViGEmEmulation();
                }
            }
        }
        catch (Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Failed to load calibration: {ex.Message}");
        }
    }

    private class CalibrationData
    {
        public double[][]? Directions { get; set; }
        public double[][]? Positions { get; set; }
    }

    private struct Point3D
    {
        public double X, Y, Z;
        public Point3D(double x, double y, double z) { X = x; Y = y; Z = z; }
    }
}