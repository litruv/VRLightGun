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

    /// <summary>
    /// Represents the state of a single controller (left or right hand).
    /// </summary>
    private class ControllerState
    {
        public bool IsAvailable { get; set; }
        public Nefarius.ViGEm.Client.Targets.IXbox360Controller? VirtualController { get; set; }
        public Valve.VR.ETrackedControllerRole Role { get; set; }
        public string Name { get; set; } = "";
        public System.Windows.Media.Color DebugColor { get; set; } = System.Windows.Media.Colors.LimeGreen;
    }

    // Controller states for left and right hands
    private readonly ControllerState _leftHand = new() 
    { 
        Role = Valve.VR.ETrackedControllerRole.LeftHand, 
        Name = "Left Hand",
        DebugColor = System.Windows.Media.Colors.Cyan
    };
    private readonly ControllerState _rightHand = new() 
    { 
        Role = Valve.VR.ETrackedControllerRole.RightHand, 
        Name = "Right Hand",
        DebugColor = System.Windows.Media.Colors.Orange
    };

    // Shared calibration data (used by all controllers)
    private CalibrationStep _calibrationStep = CalibrationStep.None;
    private readonly Point3D[] _calibrationDirections = new Point3D[3];
    private readonly Point3D[] _calibrationPositions = new Point3D[3];

    // SteamVR/OpenVR fields
    private Valve.VR.CVRSystem? _vrSystem;
    private System.Threading.CancellationTokenSource? _pollingCts;

    // IVRInput action handles
    private ulong _actionSetHandle;
    private ulong _triggerAction, _triggerValueAction;
    private ulong _gripAction, _gripValueAction;
    private ulong _aButtonAction, _bButtonAction;
    private ulong _thumbstickAction, _thumbstickClickAction;
    private ulong _trackpadAction, _trackpadTouchAction;
    private ulong _poseAction;
    private ulong _leftHandHandle, _rightHandHandle;

    // ViGEm fields
    private Nefarius.ViGEm.Client.ViGEmClient? _vigemClient;

    // 3D Debug window
    private Debug3DWindow? _debug3DWindow;
    
    // Axis debug timer
    private System.Windows.Threading.DispatcherTimer? _axisDebugTimer;

    public MainWindow()
    {
        InitializeComponent();
        this.StateChanged += MainWindow_StateChanged;
        this.Closing += MainWindow_Closing;
        
        LoadCalibration();
        StartAxisDebugTimer();
    }

    private void StartAxisDebugTimer()
    {
        _axisDebugTimer = new System.Windows.Threading.DispatcherTimer();
        // Slow this down a bit so the text is readable and doesn't flicker
        _axisDebugTimer.Interval = TimeSpan.FromMilliseconds(200);
        _axisDebugTimer.Tick += (s, e) =>
        {
            if (_vrSystem == null)
            {
                if (!InitOpenVR())
                    return;
            }

            var input = Valve.VR.OpenVR.Input;
            if (input == null || _actionSetHandle == 0)
            {
                AxisDebugBox.Text = "Input not initialized";
                return;
            }

            // Update action state
            var actionSet = new Valve.VR.VRActiveActionSet_t
            {
                ulActionSet = _actionSetHandle,
                ulRestrictedToDevice = Valve.VR.OpenVR.k_ulInvalidInputValueHandle,
                ulSecondaryActionSet = 0,
                unPadding = 0,
                nPriority = 0
            };
            var actionSets = new[] { actionSet };
            input.UpdateActionState(actionSets, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRActiveActionSet_t)));

            // Get thumbstick data (right hand) - try with 0 to not restrict
            var thumbstickData = new Valve.VR.InputAnalogActionData_t();
            var thumbErr = input.GetAnalogActionData(_thumbstickAction, ref thumbstickData, 
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), 0);

            // Get trackpad data
            var trackpadData = new Valve.VR.InputAnalogActionData_t();
            var trackErr = input.GetAnalogActionData(_trackpadAction, ref trackpadData,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), 0);

            // Get trigger value
            var triggerData = new Valve.VR.InputAnalogActionData_t();
            var triggerErr = input.GetAnalogActionData(_triggerValueAction, ref triggerData,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), 0);

            // Get grip value
            var gripData = new Valve.VR.InputAnalogActionData_t();
            var gripErr = input.GetAnalogActionData(_gripValueAction, ref gripData,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), 0);

            // Get button states
            var triggerBtn = new Valve.VR.InputDigitalActionData_t();
            var trigBtnErr = input.GetDigitalActionData(_triggerAction, ref triggerBtn,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), 0);
            var aBtn = new Valve.VR.InputDigitalActionData_t();
            input.GetDigitalActionData(_aButtonAction, ref aBtn,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), 0);
            var bBtn = new Valve.VR.InputDigitalActionData_t();
            input.GetDigitalActionData(_bButtonAction, ref bBtn,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), 0);

            AxisDebugBox.Text =
                $"Thumbstick: ({thumbstickData.x:F2}, {thumbstickData.y:F2}) err={thumbErr} active={thumbstickData.bActive}\n" +
                $"Trackpad: ({trackpadData.x:F2}, {trackpadData.y:F2}) err={trackErr} active={trackpadData.bActive}\n" +
                $"Trigger: {triggerData.x:F2} err={triggerErr} active={triggerData.bActive}\n" +
                $"Grip: {gripData.x:F2} err={gripErr} active={gripData.bActive}\n" +
                $"TrigBtn: state={(triggerBtn.bState ? "1" : "0")} active={triggerBtn.bActive} err={trigBtnErr}  A:{(aBtn.bState ? "1" : "0")} B:{(bBtn.bState ? "1" : "0")}\n" +
                $"Handles: AS={_actionSetHandle} TS={_thumbstickAction}";
        };
        _axisDebugTimer.Start();
    }

    /// <summary>
    /// Toggles the 3D debug window visibility.
    /// </summary>
    public void ToggleDebugWindow()
    {
        if (_debug3DWindow == null || !_debug3DWindow.IsLoaded)
        {
            _debug3DWindow = new Debug3DWindow();
            _debug3DWindow.Show();
        }
        else if (_debug3DWindow.IsVisible)
        {
            _debug3DWindow.Hide();
        }
        else
        {
            _debug3DWindow.Show();
            _debug3DWindow.Activate();
        }
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
        _emulationTimer?.Stop();
        _axisDebugTimer?.Stop();
        _vrSystem = null;
    }

    /// <summary>
    /// Updates the availability of controllers based on what SteamVR detects.
    /// </summary>
    private void UpdateControllerAvailability()
    {
        if (_vrSystem == null)
        {
            _leftHand.IsAvailable = false;
            _rightHand.IsAvailable = false;
            UpdateControllerStatusUI();
            return;
        }

        uint leftIndex = _vrSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.LeftHand);
        uint rightIndex = _vrSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.RightHand);

        bool leftWasAvailable = _leftHand.IsAvailable;
        bool rightWasAvailable = _rightHand.IsAvailable;

        _leftHand.IsAvailable = leftIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid;
        _rightHand.IsAvailable = rightIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid;

        // Manage virtual controllers based on availability
        if (_leftHand.IsAvailable && !leftWasAvailable && _vigemClient != null)
        {
            _leftHand.VirtualController = _vigemClient.CreateXbox360Controller();
            _leftHand.VirtualController.Connect();
        }
        else if (!_leftHand.IsAvailable && leftWasAvailable && _leftHand.VirtualController != null)
        {
            try { _leftHand.VirtualController.Disconnect(); } catch { }
            _leftHand.VirtualController = null;
        }

        if (_rightHand.IsAvailable && !rightWasAvailable && _vigemClient != null)
        {
            _rightHand.VirtualController = _vigemClient.CreateXbox360Controller();
            _rightHand.VirtualController.Connect();
        }
        else if (!_rightHand.IsAvailable && rightWasAvailable && _rightHand.VirtualController != null)
        {
            try { _rightHand.VirtualController.Disconnect(); } catch { }
            _rightHand.VirtualController = null;
        }

        UpdateControllerStatusUI();
    }

    /// <summary>
    /// Updates the UI to reflect current controller availability.
    /// </summary>
    private void UpdateControllerStatusUI()
    {
        var parts = new List<string>();
        if (_leftHand.IsAvailable) parts.Add("Left Hand");
        if (_rightHand.IsAvailable) parts.Add("Right Hand");

        if (parts.Count == 0)
            ControllerStatusText.Text = "No controllers detected";
        else
            ControllerStatusText.Text = $"Active: {string.Join(", ", parts)}";

        LeftHandDebugPanel.Visibility = _leftHand.IsAvailable ? Visibility.Visible : Visibility.Collapsed;
        RightHandDebugPanel.Visibility = _rightHand.IsAvailable ? Visibility.Visible : Visibility.Collapsed;
    }

    /// <summary>
    /// Finds an available controller for calibration.
    /// </summary>
    /// <returns>A tuple containing the controller index and name, or invalid index if none found.</returns>
    private (uint controllerIndex, string controllerName) FindCalibrationController()
    {
        var controllersToTry = new List<(Valve.VR.ETrackedControllerRole role, string name)>
        {
            (Valve.VR.ETrackedControllerRole.RightHand, "Right Hand"),
            (Valve.VR.ETrackedControllerRole.LeftHand, "Left Hand")
        };
        
        foreach (var (role, name) in controllersToTry)
        {
            uint index = _vrSystem!.GetTrackedDeviceIndexForControllerRole(role);
            if (index != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
                return (index, name);
        }
        
        return (Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid, "");
    }

    private async void StartCalibrationButton_Click(object sender, RoutedEventArgs e)
    {
        if (!InitOpenVR())
        {
            CalibrationInstructions.Text = "Failed to initialize SteamVR/OpenVR.";
            return;
        }
        
        // Stop any running emulation during calibration
        _emulationTimer?.Stop();
        
        // Find the first available controller to use for calibration
        var (calibrationController, controllerName) = FindCalibrationController();
        
        if (calibrationController == Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            CalibrationInstructions.Text = "No controller detected. Please ensure SteamVR is tracking your controllers.";
            return;
        }
        
        StartCalibrationButton.IsEnabled = false;
        _pollingCts = new System.Threading.CancellationTokenSource();
        
        // Start calibration with the found controller
        _calibrationStep = CalibrationStep.TopLeft;
        CalibrationInstructions.Text = $"[{controllerName}] Point at the TOP-LEFT corner of your screen and press the trigger.";
        
        await PollControllerForCalibration(_pollingCts.Token, calibrationController, controllerName);
        
        StartCalibrationButton.IsEnabled = true;
        
        if (_calibrationStep == CalibrationStep.Complete)
        {
            SaveCalibration();
            CalibrationInstructions.Text = "Calibration complete! All enabled controllers will use this calibration.";
            StartViGEmEmulation();
        }
    }

    private bool InitOpenVR()
    {
        if (_vrSystem != null)
            return true;
        var error = Valve.VR.EVRInitError.None;
        _vrSystem = Valve.VR.OpenVR.Init(ref error, Valve.VR.EVRApplicationType.VRApplication_Scene);
        if (error != Valve.VR.EVRInitError.None || _vrSystem == null)
        {
            System.Windows.MessageBox.Show($"Failed to init OpenVR: {error}", "VR Error");
            return false;
        }
        
        // Ensure SteamVR knows about this application and links it to our actions
        RegisterAsVRApplication();
        
        if (!InitVRInput())
        {
            System.Windows.MessageBox.Show("Failed to init VR Input actions", "VR Error");
            return false;
        }
        
        return true;
    }

        private bool InitVRInput()
        {
                var input = Valve.VR.OpenVR.Input;
                if (input == null)
                        return false;

                string? exeDir = System.IO.Path.GetDirectoryName(System.Diagnostics.Process.GetCurrentProcess().MainModule?.FileName);
                if (exeDir == null)
                        return false;

                string actionsPath = System.IO.Path.Combine(exeDir, "actions.json");
                if (!System.IO.File.Exists(actionsPath))
                {
                        System.Windows.MessageBox.Show($"actions.json not found at: {actionsPath}", "VR Error");
                        return false;
                }

                var err = input.SetActionManifestPath(actionsPath);
        if (err != Valve.VR.EVRInputError.None)
        {
            System.Windows.MessageBox.Show($"SetActionManifestPath failed: {err}", "VR Error");
            return false;
        }

        var errors = new System.Collections.Generic.List<string>();
        
        var e1 = input.GetActionSetHandle("/actions/main", ref _actionSetHandle);
        if (e1 != Valve.VR.EVRInputError.None) errors.Add($"ActionSet: {e1}");
        
        var e2 = input.GetActionHandle("/actions/main/in/Trigger", ref _triggerAction);
        if (e2 != Valve.VR.EVRInputError.None) errors.Add($"Trigger: {e2}");
        
        input.GetActionHandle("/actions/main/in/TriggerValue", ref _triggerValueAction);
        input.GetActionHandle("/actions/main/in/Grip", ref _gripAction);
        input.GetActionHandle("/actions/main/in/GripValue", ref _gripValueAction);
        input.GetActionHandle("/actions/main/in/AButton", ref _aButtonAction);
        input.GetActionHandle("/actions/main/in/BButton", ref _bButtonAction);
        
        var eThumb = input.GetActionHandle("/actions/main/in/Thumbstick", ref _thumbstickAction);
        if (eThumb != Valve.VR.EVRInputError.None) errors.Add($"Thumbstick: {eThumb}");
        
        input.GetActionHandle("/actions/main/in/ThumbstickClick", ref _thumbstickClickAction);
        input.GetActionHandle("/actions/main/in/Trackpad", ref _trackpadAction);
        input.GetActionHandle("/actions/main/in/TrackpadTouch", ref _trackpadTouchAction);
        input.GetActionHandle("/actions/main/in/Pose", ref _poseAction);

        input.GetInputSourceHandle("/user/hand/left", ref _leftHandHandle);
        input.GetInputSourceHandle("/user/hand/right", ref _rightHandHandle);

        if (errors.Count > 0)
        {
            System.Windows.MessageBox.Show($"Action handle errors:\n{string.Join("\n", errors)}", "VR Debug");
        }

        System.Diagnostics.Debug.WriteLine($"[VRInput] ActionSet={_actionSetHandle}, Thumbstick={_thumbstickAction}, RightHand={_rightHandHandle}");
        
        return _actionSetHandle != 0;
    }

        private void RegisterAsVRApplication()
        {
                var applications = Valve.VR.OpenVR.Applications;
                if (applications == null)
                {
                        return;
                }

                string? exeDir = System.IO.Path.GetDirectoryName(System.Diagnostics.Process.GetCurrentProcess().MainModule?.FileName);
                if (exeDir == null)
                        return;

                const string appKey = "com.knucklegun.lightgun";

                string manifestPath = System.IO.Path.Combine(exeDir, "knucklegun.vrmanifest");
        
                string manifestContent = $@"{{
    ""source"": ""user"",
    ""applications"": [
        {{
            ""app_key"": ""{appKey}"",
            ""launch_type"": ""binary"",
            ""binary_path_windows"": ""{System.Diagnostics.Process.GetCurrentProcess().MainModule?.FileName?.Replace("\\", "\\\\")}"",
            ""working_directory"": ""{exeDir.Replace("\\", "\\\\")}"",
            ""is_dashboard_overlay"": false,
            ""action_manifest_path"": ""actions.json"",
            ""strings"": {{
                ""en_us"": {{
                    ""name"": ""KnuckleGun VR Light Gun"",
                    ""description"": ""VR Light Gun Emulator using Index Controllers""
                }}
            }}
        }}
    ]
}}";
                System.IO.File.WriteAllText(manifestPath, manifestContent);

                // Register manifest and identify this running process; show results for debugging
                bool installed = applications.IsApplicationInstalled(appKey);
                var addResult = applications.AddApplicationManifest(manifestPath, false);
                var identifyResult = applications.IdentifyApplication((uint)System.Diagnostics.Process.GetCurrentProcess().Id, appKey);

                System.Windows.MessageBox.Show(
                        $"SteamVR app registration:\n" +
                        $"Manifest: {manifestPath}\n" +
                        $"WasInstalledBefore: {installed}\n" +
                        $"AddApplicationManifest: {addResult}\n" +
                        $"IdentifyApplication: {identifyResult}",
                        "SteamVR Registration");
        }

    private System.Windows.Threading.DispatcherTimer? _calibrationTimer;
    private uint _calibrationControllerIndex;
    private string _calibrationControllerName = "";
    private bool _waitingForTriggerRelease;

    private async System.Threading.Tasks.Task PollControllerForCalibration(System.Threading.CancellationToken token, uint controllerIndex, string controllerName)
    {
        _calibrationControllerIndex = controllerIndex;
        _calibrationControllerName = controllerName;
        _waitingForTriggerRelease = false;

        // Use a TaskCompletionSource to await calibration completion
        var tcs = new System.Threading.Tasks.TaskCompletionSource<bool>();

        _calibrationTimer = new System.Windows.Threading.DispatcherTimer();
        _calibrationTimer.Interval = TimeSpan.FromMilliseconds(16); // ~60Hz
        _calibrationTimer.Tick += (s, e) =>
        {
            if (_calibrationStep == CalibrationStep.Complete || token.IsCancellationRequested)
            {
                _calibrationTimer?.Stop();
                tcs.TrySetResult(true);
                return;
            }

            var input = Valve.VR.OpenVR.Input;
            if (input == null || _actionSetHandle == 0 || _vrSystem == null)
                return;

            // Update action state
            var actionSet = new Valve.VR.VRActiveActionSet_t
            {
                ulActionSet = _actionSetHandle,
                ulRestrictedToDevice = Valve.VR.OpenVR.k_ulInvalidInputValueHandle,
                ulSecondaryActionSet = 0,
                unPadding = 0,
                nPriority = 0
            };
            var actionSets = new[] { actionSet };
            input.UpdateActionState(actionSets, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRActiveActionSet_t)));

            // Get trigger state using IVRInput
            var triggerData = new Valve.VR.InputDigitalActionData_t();
            input.GetDigitalActionData(_triggerAction, ref triggerData,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), 0);

            bool triggerDown = triggerData.bActive && triggerData.bState;

            // Get poses
            Valve.VR.TrackedDevicePose_t[] poses = new Valve.VR.TrackedDevicePose_t[Valve.VR.OpenVR.k_unMaxTrackedDeviceCount];
            _vrSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseStanding, 0, poses);

            var pose = poses[_calibrationControllerIndex];
            if (pose.bPoseIsValid)
            {
                var m = pose.mDeviceToAbsoluteTracking;
                var controllerPos = new Point3D(m.m3, m.m7, m.m11);
                var pointingDir = GetPointingDirection(m);

                // Update debug visualization during calibration
                UpdateDebugDuringCalibration(controllerPos, pointingDir);

                // Handle trigger press/release
                if (_waitingForTriggerRelease)
                {
                    if (!triggerDown)
                        _waitingForTriggerRelease = false;
                }
                else if (triggerDown)
                {
                    RecordCalibrationPoint(pointingDir, controllerPos, _calibrationControllerName);
                    _waitingForTriggerRelease = true;
                }
            }
        };
        _calibrationTimer.Start();

        // Wait for calibration to complete
        await tcs.Task;
    }

    private void RecordCalibrationPoint(Point3D direction, Point3D position, string controllerName)
    {
        switch (_calibrationStep)
        {
            case CalibrationStep.TopLeft:
                _calibrationDirections[0] = direction;
                _calibrationPositions[0] = position;
                CalibrationInstructions.Text = $"[{controllerName}] Now point at the TOP-RIGHT corner and press the trigger.";
                _calibrationStep = CalibrationStep.TopRight;
                break;
            case CalibrationStep.TopRight:
                _calibrationDirections[1] = direction;
                _calibrationPositions[1] = position;
                CalibrationInstructions.Text = $"[{controllerName}] Now point at the BOTTOM-RIGHT corner and press the trigger.";
                _calibrationStep = CalibrationStep.BottomRight;
                break;
            case CalibrationStep.BottomRight:
                _calibrationDirections[2] = direction;
                _calibrationPositions[2] = position;
                _calibrationStep = CalibrationStep.Complete;
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
    /// Starts ViGEm emulation of virtual Xbox 360 controllers for available controllers.
    /// Each available controller gets its own virtual Xbox 360 controller, but they all
    /// use the same shared calibration data.
    /// </summary>
    private System.Windows.Threading.DispatcherTimer? _emulationTimer;

    private void StartViGEmEmulation()
    {
        // Stop the debug timer - emulation timer will handle input now
        _axisDebugTimer?.Stop();

        if (_vigemClient == null)
        {
            _vigemClient = new Nefarius.ViGEm.Client.ViGEmClient();
        }

        // Use a DispatcherTimer so OpenVR calls stay on the UI thread (where Init was called)
        _emulationTimer = new System.Windows.Threading.DispatcherTimer();
        _emulationTimer.Interval = TimeSpan.FromMilliseconds(8); // ~120Hz polling
        _emulationTimer.Tick += EmulationTimer_Tick;
        _emulationTimer.Start();
    }

    private void EmulationTimer_Tick(object? sender, EventArgs e)
    {
        if (_vrSystem == null)
        {
            _emulationTimer?.Stop();
            return;
        }

        var input = Valve.VR.OpenVR.Input;
        if (input == null || _actionSetHandle == 0)
            return;

        var actionSet = new Valve.VR.VRActiveActionSet_t
        {
            ulActionSet = _actionSetHandle,
            ulRestrictedToDevice = Valve.VR.OpenVR.k_ulInvalidInputValueHandle,
            ulSecondaryActionSet = 0,
            unPadding = 0,
            nPriority = 0
        };
        var actionSets = new[] { actionSet };
        input.UpdateActionState(actionSets, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRActiveActionSet_t)));

        // Update controller availability dynamically
        UpdateControllerAvailability();

        Valve.VR.TrackedDevicePose_t[] poses = new Valve.VR.TrackedDevicePose_t[Valve.VR.OpenVR.k_unMaxTrackedDeviceCount];
        _vrSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseStanding, 0, poses);

        var debugDataList = new List<ControllerDebugData>();

        foreach (var controller in new[] { _leftHand, _rightHand })
        {
            if (!controller.IsAvailable || controller.VirtualController == null)
                continue;

            uint controllerIndex = _vrSystem.GetTrackedDeviceIndexForControllerRole(controller.Role);
            if (controllerIndex == Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
                continue;

            var debugData = ProcessControllerInput(controller, controllerIndex, poses, input);
            if (debugData != null)
                debugDataList.Add(debugData);
        }

        // Update 3D debug window with all controllers
        if (_debug3DWindow != null && debugDataList.Count > 0)
        {
            var calibPositions = new System.Windows.Media.Media3D.Point3D[]
            {
                new(_calibrationPositions[0].X, _calibrationPositions[0].Y, _calibrationPositions[0].Z),
                new(_calibrationPositions[1].X, _calibrationPositions[1].Y, _calibrationPositions[1].Z),
                new(_calibrationPositions[2].X, _calibrationPositions[2].Y, _calibrationPositions[2].Z)
            };
            _debug3DWindow?.UpdateVisualization(calibPositions, debugDataList);
        }
    }

    /// <summary>
    /// Processes input from a single controller and updates its virtual controller.
    /// Uses the shared calibration data for screen coordinate calculation and IVRInput actions for buttons/axes.
    /// Returns debug data for visualization.
    /// </summary>
    private ControllerDebugData? ProcessControllerInput(ControllerState controller, uint controllerIndex, Valve.VR.TrackedDevicePose_t[] poses, Valve.VR.CVRInput input)
    {
        // Per-hand input source handle
        ulong sourceHandle = controller.Role == Valve.VR.ETrackedControllerRole.LeftHand
            ? _leftHandHandle
            : _rightHandHandle;

        // Button states
        bool triggerPressed = false;
        bool gripPressed = false;
        bool aPressed = false;
        bool bPressed = false;
        bool thumbstickPressed = false;
        bool trackpadTouched = false;
        
        // Analog values
        float triggerValue = 0f;
        float gripValue = 0f;
        float trackpadX = 0f, trackpadY = 0f;
        float thumbstickX = 0f, thumbstickY = 0f;

        // Read analog actions for this hand
        var analog = new Valve.VR.InputAnalogActionData_t();

        if (input.GetAnalogActionData(_triggerValueAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            triggerValue = analog.x;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_gripValueAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            gripValue = analog.x;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_thumbstickAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            thumbstickX = analog.x;
            thumbstickY = analog.y;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_trackpadAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            trackpadX = analog.x;
            trackpadY = analog.y;
        }

        // Read digital actions for this hand
        var digital = new Valve.VR.InputDigitalActionData_t();

        if (input.GetDigitalActionData(_triggerAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            triggerPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_gripAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            gripPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_aButtonAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            aPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_bButtonAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            bPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_thumbstickClickAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            thumbstickPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_trackpadTouchAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            trackpadTouched = digital.bState;
        }

        var pose = poses[controllerIndex];
        if (!pose.bPoseIsValid)
            return null;

        var m = pose.mDeviceToAbsoluteTracking;

        // Get current pointing direction with -45° pitch
        var dir = GetPointingDirection(m);
        var controllerPos = new Point3D(m.m3, m.m7, m.m11);

        // Use ray-plane intersection to find screen coordinates
        var TL = _calibrationPositions[0];
        var TR = _calibrationPositions[1];
        var BR = _calibrationPositions[2];

        // Calculate screen axes from calibration positions
        double screenXx = TR.X - TL.X;
        double screenXy = TR.Y - TL.Y;
        double screenXz = TR.Z - TL.Z;
        
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

        double denom = normalX * dir.X + normalY * dir.Y + normalZ * dir.Z;
        
        float xNorm = 0.5f, yNorm = 0.5f;
        if (Math.Abs(denom) > 1e-9)
        {
            double toPlaneX = TL.X - controllerPos.X;
            double toPlaneY = TL.Y - controllerPos.Y;
            double toPlaneZ = TL.Z - controllerPos.Z;
            double t = (normalX * toPlaneX + normalY * toPlaneY + normalZ * toPlaneZ) / denom;

            if (t > 0)
            {
                double hitX = controllerPos.X + t * dir.X;
                double hitY = controllerPos.Y + t * dir.Y;
                double hitZ = controllerPos.Z + t * dir.Z;

                double offsetX = hitX - TL.X;
                double offsetY = hitY - TL.Y;
                double offsetZ = hitZ - TL.Z;

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

        xNorm = Math.Clamp(xNorm, 0f, 1f);
        yNorm = Math.Clamp(yNorm, 0f, 1f);

        short stickX = (short)(xNorm * 65535 - 32768);
        short stickY = (short)((1f - yNorm) * 65535 - 32768);

        // Update debug text boxes on UI thread
        Dispatcher.Invoke(() =>
        {
            if (controller.Role == Valve.VR.ETrackedControllerRole.LeftHand)
            {
                DebugLeftXBox.Text = xNorm.ToString("F3");
                DebugLeftYBox.Text = yNorm.ToString("F3");
            }
            else
            {
                DebugXBox.Text = xNorm.ToString("F3");
                DebugYBox.Text = yNorm.ToString("F3");
            }
        });

        // Apply absolute position to left stick (screen aiming)
        controller.VirtualController!.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbX, stickX);
        controller.VirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbY, stickY);

        // Map thumbstick to right stick
        short rightStickX = (short)(Math.Clamp(thumbstickX, -1f, 1f) * 32767);
        short rightStickY = (short)(Math.Clamp(thumbstickY, -1f, 1f) * 32767);
        controller.VirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.RightThumbX, rightStickX);
        controller.VirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.RightThumbY, rightStickY);

        // Map touchpad to D-Pad (cardinal directions when touched)
        bool dpadUp = trackpadTouched && trackpadY > 0.5f;
        bool dpadDown = trackpadTouched && trackpadY < -0.5f;
        bool dpadLeft = trackpadTouched && trackpadX < -0.5f;
        bool dpadRight = trackpadTouched && trackpadX > 0.5f;
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Up, dpadUp);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Down, dpadDown);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Left, dpadLeft);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Right, dpadRight);

        // Map triggers: primary trigger -> right trigger, grip -> left trigger
        byte rightTrigger = (byte)(Math.Clamp(triggerValue, 0f, 1f) * 255f);
        byte leftTrigger = (byte)(Math.Clamp(gripValue, 0f, 1f) * 255f);
        controller.VirtualController.SetSliderValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider.RightTrigger, rightTrigger);
        controller.VirtualController.SetSliderValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider.LeftTrigger, leftTrigger);

        // Map VR buttons to Xbox 360 buttons
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.A, aPressed);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.B, bPressed);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.X, gripPressed);
        controller.VirtualController.SetButtonState(
            Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Y,
            trackpadTouched && Math.Abs(trackpadX) < 0.5f && Math.Abs(trackpadY) < 0.5f);

        // Thumbstick click -> right stick click
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.RightThumb, thumbstickPressed);

        // Shoulders / start / back can be assigned later if needed
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.LeftShoulder, false);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.RightShoulder, false);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Start, false);
        controller.VirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Back, false);

        controller.VirtualController.SubmitReport();

        // Return debug data for visualization
        return new ControllerDebugData
        {
            Name = controller.Name,
            Position = new System.Windows.Media.Media3D.Point3D(controllerPos.X, controllerPos.Y, controllerPos.Z),
            Direction = new System.Windows.Media.Media3D.Vector3D(dir.X, dir.Y, dir.Z),
            XNorm = xNorm,
            YNorm = yNorm,
            Color = controller.DebugColor
        };
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
                _calibrationStep = CalibrationStep.Complete;
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
