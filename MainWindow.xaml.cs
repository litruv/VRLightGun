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
    /// Controller operation mode.
    /// </summary>
    private enum ControllerMode
    {
        LightGun,
        SplitXboxController
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
    private ulong _aButtonTouchAction, _bButtonTouchAction;
    private ulong _thumbstickAction, _thumbstickClickAction;
    private ulong _trackpadAction, _trackpadTouchAction;
    private ulong _poseAction;
    private ulong _leftHandHandle, _rightHandHandle;

    // ViGEm fields
    private Nefarius.ViGEm.Client.ViGEmClient? _vigemClient;
    private Nefarius.ViGEm.Client.Targets.IXbox360Controller? _sharedVirtualController;

    // Controller mode
    private ControllerMode _currentMode = ControllerMode.LightGun;
    
    // Split controller mode settings
    private float _stickDeadzone = 0.1f;
    private float _gyroDeadzone = 0.05f;
    private float _gyroScale = 0.5f;
    private float _leftOrientScale = 0.5f;

    // Left controller orientation-based stick: reference pose captured on thumb touch start
    private Valve.VR.HmdMatrix34_t? _leftGyroRefMatrix = null;
    private bool _leftWasTouching = false;

    // Live debug values for left orientation stick
    private float _dbgLeftStickX, _dbgLeftStickY;
    
    // Flag to prevent double-initialization during load
    private bool _isSwitchingModes = false;

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
        
        // Wire up save-triggering events AFTER loading so XAML init doesn't cause spurious saves
        ControllerModeComboBox.SelectionChanged += ControllerModeComboBox_SelectionChanged;
        StickDeadzoneSlider.ValueChanged += StickDeadzoneSlider_ValueChanged;
        GyroDeadzoneSlider.ValueChanged += GyroDeadzoneSlider_ValueChanged;
        GyroScaleSlider.ValueChanged += GyroScaleSlider_ValueChanged;
        LeftOrientScaleSlider.ValueChanged += LeftOrientScaleSlider_ValueChanged;
        
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
                $"Handles: AS={_actionSetHandle} TS={_thumbstickAction}\n" +
                $"--- Left orient-stick (touch thumb to show) ---\n" +
                $"StickX:{_dbgLeftStickX,7:F3}  StickY:{_dbgLeftStickY,7:F3}";
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
        System.Diagnostics.Debug.WriteLine($"[MainWindow_Closing] Called. ShutdownMode={System.Windows.Application.Current?.ShutdownMode}");
        
        // Hide instead of close, unless app is shutting down
        if (System.Windows.Application.Current?.ShutdownMode != ShutdownMode.OnExplicitShutdown)
        {
            System.Diagnostics.Debug.WriteLine("[MainWindow_Closing] Hiding window (not shutting down)");
            e.Cancel = true;
            this.Hide();
            this.ShowInTaskbar = false;
            return;
        }

        // Only run cleanup if actually shutting down
        System.Diagnostics.Debug.WriteLine("[MainWindow_Closing] Running cleanup (shutting down)");
        _pollingCts?.Cancel();
        _emulationTimer?.Stop();
        _axisDebugTimer?.Stop();
        CleanupVirtualControllers();
        _vrSystem = null;
    }

    /// <summary>
    /// Handles controller mode selection changes.
    /// </summary>
    private void ControllerModeComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (ModeDescription == null || CalibrationInstructions == null)
            return;

        System.Diagnostics.Debug.WriteLine($"[ControllerModeComboBox_SelectionChanged] Index={ControllerModeComboBox.SelectedIndex}");

        bool wasRunning = _emulationTimer != null && _emulationTimer.IsEnabled;

        if (ControllerModeComboBox.SelectedIndex == 0)
        {
            _currentMode = ControllerMode.LightGun;
            ModeDescription.Text = "Point controllers at screen to aim. Each controller gets its own virtual Xbox controller.";
            SplitModeSettings.Visibility = Visibility.Collapsed;
            CalibrationSection.Visibility = Visibility.Visible;
        }
        else
        {
            _currentMode = ControllerMode.SplitXboxController;
            ModeDescription.Text = "Both controllers control one Xbox controller. Left controller → left stick, right controller → right stick. Gyro active when touching A/B/trackpad.";
            SplitModeSettings.Visibility = Visibility.Visible;
            CalibrationSection.Visibility = Visibility.Collapsed;
        }

        if (wasRunning)
        {
            _isSwitchingModes = true;
            _emulationTimer!.Stop();
            CleanupVirtualControllers();
            
            var restartTimer = new System.Windows.Threading.DispatcherTimer();
            restartTimer.Interval = TimeSpan.FromSeconds(1);
            restartTimer.Tick += (s, evt) =>
            {
                restartTimer.Stop();
                _isSwitchingModes = false;
                if (InitOpenVR())
                    StartViGEmEmulation();
            };
            restartTimer.Start();
        }
        else if (_currentMode == ControllerMode.SplitXboxController)
        {
            if (InitOpenVR())
                StartViGEmEmulation();
        }
        
        SaveCalibration();
    }

    private void StickDeadzoneSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        _stickDeadzone = (float)e.NewValue;
        if (StickDeadzoneValue != null)
            StickDeadzoneValue.Text = _stickDeadzone.ToString("F2");
        SaveCalibration();
    }

    private void GyroDeadzoneSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        _gyroDeadzone = (float)e.NewValue;
        if (GyroDeadzoneValue != null)
            GyroDeadzoneValue.Text = _gyroDeadzone.ToString("F2");
        SaveCalibration();
    }

    private void GyroScaleSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        _gyroScale = (float)e.NewValue;
        if (GyroScaleValue != null)
            GyroScaleValue.Text = _gyroScale.ToString("F2");
        SaveCalibration();
    }

    /// <summary>
    /// Handles left orientation sensitivity slider changes.
    /// </summary>
    private void LeftOrientScaleSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        _leftOrientScale = (float)e.NewValue;
        if (LeftOrientScaleValue != null)
            LeftOrientScaleValue.Text = _leftOrientScale.ToString("F2");
        SaveCalibration();
    }

    /// <summary>
    /// Cleans up all virtual controllers.
    /// </summary>
    private void CleanupVirtualControllers()
    {
        if (_leftHand.VirtualController != null)
        {
            try { _leftHand.VirtualController.Disconnect(); } catch { }
            _leftHand.VirtualController = null;
        }
        if (_rightHand.VirtualController != null)
        {
            try { _rightHand.VirtualController.Disconnect(); } catch { }
            _rightHand.VirtualController = null;
        }
        if (_sharedVirtualController != null)
        {
            try { _sharedVirtualController.Disconnect(); } catch { }
            _sharedVirtualController = null;
        }
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
        
        System.Diagnostics.Debug.WriteLine($"[UpdateControllerAvailability] LeftIdx={leftIndex}, RightIdx={rightIndex}, Mode={_currentMode}");

        bool leftWasAvailable = _leftHand.IsAvailable;
        bool rightWasAvailable = _rightHand.IsAvailable;

        _leftHand.IsAvailable = leftIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid;
        _rightHand.IsAvailable = rightIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid;
        
        System.Diagnostics.Debug.WriteLine($"[UpdateControllerAvailability] Left={_leftHand.IsAvailable}, Right={_rightHand.IsAvailable}");

        // Manage virtual controllers based on mode and availability
        if (_currentMode == ControllerMode.SplitXboxController)
        {
            // In split mode, use a single shared controller
            bool anyAvailable = _leftHand.IsAvailable || _rightHand.IsAvailable;

            // Create shared controller if any controller is available and we don't have one yet
            if (anyAvailable && _sharedVirtualController == null && _vigemClient != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Creating shared virtual controller");
                _sharedVirtualController = _vigemClient.CreateXbox360Controller();
                _sharedVirtualController.Connect();
            }
            else if (!anyAvailable && _sharedVirtualController != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Disconnecting shared virtual controller");
                try { _sharedVirtualController.Disconnect(); } catch { }
                _sharedVirtualController = null;
            }

            // Clear individual controllers in split mode
            if (_leftHand.VirtualController != null)
            {
                try { _leftHand.VirtualController.Disconnect(); } catch { }
                _leftHand.VirtualController = null;
            }
            if (_rightHand.VirtualController != null)
            {
                try { _rightHand.VirtualController.Disconnect(); } catch { }
                _rightHand.VirtualController = null;
            }
        }
        else
        {
            // In light gun mode, each controller gets its own virtual controller
            if (_leftHand.IsAvailable && _leftHand.VirtualController == null && _vigemClient != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Creating left virtual controller");
                _leftHand.VirtualController = _vigemClient.CreateXbox360Controller();
                _leftHand.VirtualController.Connect();
            }
            else if (!_leftHand.IsAvailable && _leftHand.VirtualController != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Disconnecting left virtual controller");
                try { _leftHand.VirtualController.Disconnect(); } catch { }
                _leftHand.VirtualController = null;
            }

            if (_rightHand.IsAvailable && _rightHand.VirtualController == null && _vigemClient != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Creating right virtual controller");
                _rightHand.VirtualController = _vigemClient.CreateXbox360Controller();
                _rightHand.VirtualController.Connect();
            }
            else if (!_rightHand.IsAvailable && _rightHand.VirtualController != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Disconnecting right virtual controller");
                try { _rightHand.VirtualController.Disconnect(); } catch { }
                _rightHand.VirtualController = null;
            }

            // Clear shared controller in light gun mode
            if (_sharedVirtualController != null)
            {
                System.Diagnostics.Debug.WriteLine("[UpdateControllerAvailability] Disconnecting shared virtual controller in light gun mode");
                try { _sharedVirtualController.Disconnect(); } catch { }
                _sharedVirtualController = null;
            }
        }

        UpdateControllerStatusUI();
    }

    /// <summary>
    /// Updates the UI to reflect current controller availability.
    /// </summary>
    private void UpdateControllerStatusUI()
    {
        Dispatcher.Invoke(() =>
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
        });
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
        input.GetActionHandle("/actions/main/in/AButtonTouch", ref _aButtonTouchAction);
        input.GetActionHandle("/actions/main/in/BButton", ref _bButtonAction);
        input.GetActionHandle("/actions/main/in/BButtonTouch", ref _bButtonTouchAction);
        
        var eThumb = input.GetActionHandle("/actions/main/in/Thumbstick", ref _thumbstickAction);
        if (eThumb != Valve.VR.EVRInputError.None) errors.Add($"Thumbstick: {eThumb}");
        
        input.GetActionHandle("/actions/main/in/ThumbstickClick", ref _thumbstickClickAction);
        input.GetActionHandle("/actions/main/in/Trackpad", ref _trackpadAction);
        input.GetActionHandle("/actions/main/in/TrackpadTouch", ref _trackpadTouchAction);
        input.GetActionHandle("/actions/main/in/Pose", ref _poseAction);

        input.GetInputSourceHandle("/user/hand/left", ref _leftHandHandle);
        input.GetInputSourceHandle("/user/hand/right", ref _rightHandHandle);

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

                // Register manifest and identify this running process
                bool installed = applications.IsApplicationInstalled(appKey);
                var addResult = applications.AddApplicationManifest(manifestPath, false);
                var identifyResult = applications.IdentifyApplication((uint)System.Diagnostics.Process.GetCurrentProcess().Id, appKey);
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

        if (_currentMode == ControllerMode.SplitXboxController)
        {
            // In split mode, process both controllers together and merge to shared controller
            ProcessSplitControllerInput(poses, input, debugDataList);
        }
        else
        {
            // In light gun mode, process each controller individually
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
    /// Processes input from both controllers in split Xbox controller mode.
    /// Merges inputs to a single shared virtual controller.
    /// </summary>
    private void ProcessSplitControllerInput(Valve.VR.TrackedDevicePose_t[] poses, Valve.VR.CVRInput input, List<ControllerDebugData> debugDataList)
    {
        if (_sharedVirtualController == null || _vrSystem == null)
            return;

        // Get controller indices
        uint leftIndex = _vrSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.LeftHand);
        uint rightIndex = _vrSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.RightHand);

        // Initialize default values
        float leftStickX = 0f, leftStickY = 0f;
        float rightStickX = 0f, rightStickY = 0f;
        bool dpadUp = false, dpadDown = false, dpadLeft = false, dpadRight = false;
        bool leftTriggerPressed = false, rightTriggerPressed = false;
        bool leftGripPressed = false, rightGripPressed = false;
        bool aPressed = false, bPressed = false;
        bool xPressed = false, yPressed = false;
        bool leftThumbPressed = false, rightThumbPressed = false;
        float leftTriggerValue = 0f, rightTriggerValue = 0f;
        float leftGripValue = 0f, rightGripValue = 0f;
        bool leftThumbTouching = false, rightThumbTouching = false;
        System.Windows.Media.Media3D.Vector3D leftGyro = new(), rightGyro = new();

        // Process left controller
        if (_leftHand.IsAvailable && leftIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            var leftData = GetControllerInputData(leftIndex, _leftHandHandle, poses, input);
            if (leftData != null)
            {
                leftStickX = leftData.Value.thumbstickX;
                leftStickY = leftData.Value.thumbstickY;
                leftThumbPressed = leftData.Value.thumbstickPressed;
                leftGripPressed = leftData.Value.gripPressed;
                leftTriggerPressed = leftData.Value.triggerPressed;
                leftTriggerValue = leftData.Value.triggerValue;
                leftGripValue = leftData.Value.gripValue;
                xPressed = leftData.Value.aPressed;
                yPressed = leftData.Value.bPressed;
                leftThumbTouching = leftData.Value.thumbTouching;
                leftGyro = leftData.Value.angularVelocity;

                // Left trackpad contributes to D-pad
                if (leftData.Value.trackpadTouched)
                {
                    if (leftData.Value.trackpadY > 0.5f) dpadUp = true;
                    if (leftData.Value.trackpadY < -0.5f) dpadDown = true;
                    if (leftData.Value.trackpadX < -0.5f) dpadLeft = true;
                    if (leftData.Value.trackpadX > 0.5f) dpadRight = true;
                }
            }
        }

        // Process right controller
        if (_rightHand.IsAvailable && rightIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            var rightData = GetControllerInputData(rightIndex, _rightHandHandle, poses, input);
            if (rightData != null)
            {
                rightStickX = rightData.Value.thumbstickX;
                rightStickY = rightData.Value.thumbstickY;
                rightThumbPressed = rightData.Value.thumbstickPressed;
                rightGripPressed = rightData.Value.gripPressed;
                rightTriggerPressed = rightData.Value.triggerPressed;
                rightTriggerValue = rightData.Value.triggerValue;
                rightGripValue = rightData.Value.gripValue;
                aPressed = rightData.Value.aPressed;
                bPressed = rightData.Value.bPressed;
                rightThumbTouching = rightData.Value.thumbTouching;
                rightGyro = rightData.Value.angularVelocity;

                // Right trackpad contributes to D-pad
                if (rightData.Value.trackpadTouched)
                {
                    if (rightData.Value.trackpadY > 0.5f) dpadUp = true;
                    if (rightData.Value.trackpadY < -0.5f) dpadDown = true;
                    if (rightData.Value.trackpadX < -0.5f) dpadLeft = true;
                    if (rightData.Value.trackpadX > 0.5f) dpadRight = true;
                }
            }
        }

        // Apply stick deadzone
        if (Math.Abs(leftStickX) < _stickDeadzone) leftStickX = 0f;
        if (Math.Abs(leftStickY) < _stickDeadzone) leftStickY = 0f;
        if (Math.Abs(rightStickX) < _stickDeadzone) rightStickX = 0f;
        if (Math.Abs(rightStickY) < _stickDeadzone) rightStickY = 0f;
        
        // Apply gyro based on which controller has capacitive touch active
        // Left controller: orientation delta from reference pose captured on touch start.
        // Tilting the controller relative to its neutral position drives the stick directly.
        if (leftThumbTouching && leftIndex != Valve.VR.OpenVR.k_unTrackedDeviceIndexInvalid && poses[leftIndex].bPoseIsValid)
        {
            // Capture reference orientation the moment the thumb touches
            if (!_leftWasTouching)
                _leftGyroRefMatrix = poses[leftIndex].mDeviceToAbsoluteTracking;

            if (_leftGyroRefMatrix.HasValue)
            {
                var rm = _leftGyroRefMatrix.Value;
                var pm = poses[leftIndex].mDeviceToAbsoluteTracking;

                // Reference axes from the pose at touch start
                // right  = col0: (m0, m4, m8)
                // fwd    = -col2: (-m2, -m6, -m10)  (OpenVR col2 is "back")
                float refRightX = rm.m0, refRightY = rm.m4, refRightZ = rm.m8;
                float refFwdX   = -rm.m2, refFwdY = -rm.m6, refFwdZ = -rm.m10;

                // Flatten both axes onto the horizontal plane (drop Y/vertical component)
                // so leaning your whole body doesn't skew the stick.
                refRightY = 0f;
                refFwdY   = 0f;

                // Normalise after flattening
                float rLen = MathF.Sqrt(refRightX * refRightX + refRightZ * refRightZ);
                float fLen = MathF.Sqrt(refFwdX   * refFwdX   + refFwdZ   * refFwdZ);
                if (rLen > 0.001f) { refRightX /= rLen; refRightZ /= rLen; }
                if (fLen > 0.001f) { refFwdX   /= fLen; refFwdZ   /= fLen; }

                // World-space displacement since touch start (X = right, Y = up, Z = forward/back in OpenVR)
                float dx = pm.m3  - rm.m3;
                float dy = pm.m7  - rm.m7;   // vertical – not used
                float dz = pm.m11 - rm.m11;

                // Project onto reference right and forward (horizontal plane only)
                float projX =  dx * refRightX + dz * refRightZ;
                float projY =  dx * refFwdX   + dz * refFwdZ;

                // Apply deadzone
                if (Math.Abs(projX) < _gyroDeadzone * 0.05f) projX = 0f;
                if (Math.Abs(projY) < _gyroDeadzone * 0.05f) projY = 0f;

                // Scale: _leftOrientScale * 8 → at default 0.5, ~25 cm = full deflection
                float orientX = Math.Clamp(projX * _leftOrientScale * 8f, -1f, 1f);
                float orientY = Math.Clamp(projY * _leftOrientScale * 8f, -1f, 1f);

                // Thumbstick overrides orientation when pushed past deadzone
                bool stickActive = Math.Abs(leftStickX) > _stickDeadzone || Math.Abs(leftStickY) > _stickDeadzone;
                if (!stickActive)
                {
                    leftStickX = orientX;
                    leftStickY = orientY;
                }

                _dbgLeftStickX = leftStickX;
                _dbgLeftStickY = leftStickY;
            }
        }
        else if (!leftThumbTouching)
        {
            // Reset reference when thumb lifts so next touch starts fresh
            _leftGyroRefMatrix = null;
            _dbgLeftStickX = _dbgLeftStickY = 0f;
        }
        _leftWasTouching = leftThumbTouching;

        // Right gyro: direct velocity → stick (great for aiming)
        if (rightThumbTouching)
        {
            float gyroX = (float)rightGyro.X;
            float gyroY = (float)rightGyro.Y;
            
            // Apply deadzone
            if (Math.Abs(gyroX) < _gyroDeadzone) gyroX = 0f;
            if (Math.Abs(gyroY) < _gyroDeadzone) gyroY = 0f;
            
            // Apply right controller gyro to right stick (Y gyro = horizontal aim, X gyro = vertical aim)
            rightStickX = Math.Clamp(rightStickX - gyroY * _gyroScale, -1f, 1f);
            rightStickY = Math.Clamp(rightStickY + gyroX * _gyroScale, -1f, 1f);
            
            if (Math.Abs(gyroX) > _gyroDeadzone || Math.Abs(gyroY) > _gyroDeadzone)
            {
                System.Diagnostics.Debug.WriteLine($"[Gyro] Right controller gyro=({gyroX:F3}, {gyroY:F3}) -> rightStick=({rightStickX:F3}, {rightStickY:F3})");
            }
        }

        // Convert stick values to Xbox format
        short leftStickXShort = (short)(Math.Clamp(leftStickX, -1f, 1f) * 32767);
        short leftStickYShort = (short)(Math.Clamp(leftStickY, -1f, 1f) * 32767);
        short rightStickXShort = (short)(Math.Clamp(rightStickX, -1f, 1f) * 32767);
        short rightStickYShort = (short)(Math.Clamp(rightStickY, -1f, 1f) * 32767);

        // Set axis values
        _sharedVirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbX, leftStickXShort);
        _sharedVirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.LeftThumbY, leftStickYShort);
        _sharedVirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.RightThumbX, rightStickXShort);
        _sharedVirtualController.SetAxisValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis.RightThumbY, rightStickYShort);

        // Set D-pad
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Up, dpadUp);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Down, dpadDown);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Left, dpadLeft);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Right, dpadRight);

        // Set triggers (only from trigger, not grip)
        byte leftTriggerByte = (byte)(Math.Clamp(leftTriggerValue, 0f, 1f) * 255f);
        byte rightTriggerByte = (byte)(Math.Clamp(rightTriggerValue, 0f, 1f) * 255f);
        _sharedVirtualController.SetSliderValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider.LeftTrigger, leftTriggerByte);
        _sharedVirtualController.SetSliderValue(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider.RightTrigger, rightTriggerByte);

        // Set buttons
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.A, aPressed);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.B, bPressed);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.X, xPressed);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Y, yPressed);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.LeftThumb, leftThumbPressed);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.RightThumb, rightThumbPressed);

        // Shoulders (grip buttons when squeezed past 0.25)
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.LeftShoulder, leftGripValue > 0.25f);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.RightShoulder, rightGripValue > 0.25f);

        // Start / Back
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Start, false);
        _sharedVirtualController.SetButtonState(Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Button.Back, false);

        _sharedVirtualController.SubmitReport();
    }

    /// <summary>
    /// Gets controller input data for split mode processing.
    /// </summary>
    private struct ControllerInputData
    {
        public float thumbstickX, thumbstickY;
        public float trackpadX, trackpadY;
        public bool thumbstickPressed, trackpadTouched;
        public bool triggerPressed, gripPressed;
        public bool aPressed, bPressed;
        public bool aTouched, bTouched;
        public float triggerValue, gripValue;
        public bool thumbTouching;
        public System.Windows.Media.Media3D.Vector3D angularVelocity;
    }

    /// <summary>
    /// Reads input data from a controller.
    /// </summary>
    private ControllerInputData? GetControllerInputData(uint controllerIndex, ulong sourceHandle, Valve.VR.TrackedDevicePose_t[] poses, Valve.VR.CVRInput input)
    {
        var pose = poses[controllerIndex];
        if (!pose.bPoseIsValid)
            return null;

        var data = new ControllerInputData();

        // Read analog actions
        var analog = new Valve.VR.InputAnalogActionData_t();

        if (input.GetAnalogActionData(_thumbstickAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            data.thumbstickX = analog.x;
            data.thumbstickY = analog.y;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_trackpadAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            data.trackpadX = analog.x;
            data.trackpadY = analog.y;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_triggerValueAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            data.triggerValue = analog.x;
        }

        analog = new Valve.VR.InputAnalogActionData_t();
        if (input.GetAnalogActionData(_gripValueAction, ref analog,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputAnalogActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && analog.bActive)
        {
            data.gripValue = analog.x;
        }

        // Read digital actions
        var digital = new Valve.VR.InputDigitalActionData_t();

        if (input.GetDigitalActionData(_triggerAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.triggerPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_gripAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.gripPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_aButtonAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.aPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_aButtonTouchAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.aTouched = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_bButtonAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.bPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_bButtonTouchAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.bTouched = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_thumbstickClickAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.thumbstickPressed = digital.bState;
        }

        digital = new Valve.VR.InputDigitalActionData_t();
        if (input.GetDigitalActionData(_trackpadTouchAction, ref digital,
                (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.InputDigitalActionData_t)), sourceHandle)
            == Valve.VR.EVRInputError.None && digital.bActive)
        {
            data.trackpadTouched = digital.bState;
        }

        // Gyro is active when any capacitive touch sensor is touched (A, B, or trackpad)
        data.thumbTouching = data.trackpadTouched || data.aTouched || data.bTouched;

        // Transform world-space angular velocity into the controller's local frame so that
        // each axis always maps to the same physical tilt regardless of controller orientation.
        // Pose matrix columns are the controller's local axes expressed in world space:
        //   right (X):   m0, m4, m8
        //   up    (Y):   m1, m5, m9
        //   fwd   (Z):   m2, m6, m10
        var pm = pose.mDeviceToAbsoluteTracking;
        float wx = pose.vAngularVelocity.v0;
        float wy = pose.vAngularVelocity.v1;
        float wz = pose.vAngularVelocity.v2;
        data.angularVelocity = new System.Windows.Media.Media3D.Vector3D(
            wx * pm.m0 + wy * pm.m4 + wz * pm.m8,   // local X (pitch around right axis)
            wx * pm.m1 + wy * pm.m5 + wz * pm.m9,   // local Y (yaw around up axis)
            wx * pm.m2 + wy * pm.m6 + wz * pm.m10); // local Z (roll around fwd axis)

        return data;
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
            // Ensure mode matches the UI selection
            if (ControllerModeComboBox != null)
            {
                _currentMode = (ControllerMode)ControllerModeComboBox.SelectedIndex;
            }
            
            var data = new CalibrationData
            {
                Directions = _calibrationDirections.Select(p => new double[] { p.X, p.Y, p.Z }).ToArray(),
                Positions = _calibrationPositions.Select(p => new double[] { p.X, p.Y, p.Z }).ToArray(),
                ControllerMode = (int)_currentMode,
                StickDeadzone = _stickDeadzone,
                GyroDeadzone = _gyroDeadzone,
                GyroScale = _gyroScale,
                LeftOrientScale = _leftOrientScale
            };
            var json = JsonSerializer.Serialize(data, new JsonSerializerOptions { WriteIndented = true });
            System.IO.File.WriteAllText(GetCalibrationFilePath(), json);
            System.Diagnostics.Debug.WriteLine($"[SaveCalibration] Saved ControllerMode={data.ControllerMode}, StickDeadzone={data.StickDeadzone}, GyroDeadzone={data.GyroDeadzone}, GyroScale={data.GyroScale}, LeftOrientScale={data.LeftOrientScale}");
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
            {
                System.Diagnostics.Debug.WriteLine("[LoadCalibration] File does not exist");
                return;
            }

            System.Diagnostics.Debug.WriteLine("[LoadCalibration] Loading settings...");
            
            var json = System.IO.File.ReadAllText(path);
            var data = JsonSerializer.Deserialize<CalibrationData>(json);
            if (data != null)
            {
                System.Diagnostics.Debug.WriteLine($"[LoadCalibration] Loaded ControllerMode={data.ControllerMode}, StickDeadzone={data.StickDeadzone}, GyroDeadzone={data.GyroDeadzone}, GyroScale={data.GyroScale}");
                
                if (data.Directions != null && data.Positions != null && data.Directions.Length == 3 && data.Positions.Length == 3)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        _calibrationDirections[i] = new Point3D(data.Directions[i][0], data.Directions[i][1], data.Directions[i][2]);
                        _calibrationPositions[i] = new Point3D(data.Positions[i][0], data.Positions[i][1], data.Positions[i][2]);
                    }
                    _calibrationStep = CalibrationStep.Complete;
                    CalibrationInstructions.Text = "Calibration loaded from file.";
                }
                
                _currentMode = (ControllerMode)data.ControllerMode;
                ControllerModeComboBox.SelectedIndex = (int)_currentMode;
                
                _stickDeadzone = data.StickDeadzone;
                _gyroDeadzone = data.GyroDeadzone;
                _gyroScale = data.GyroScale;
                _leftOrientScale = data.LeftOrientScale;
                
                StickDeadzoneSlider.Value = _stickDeadzone;
                GyroDeadzoneSlider.Value = _gyroDeadzone;
                GyroScaleSlider.Value = _gyroScale;
                LeftOrientScaleSlider.Value = _leftOrientScale;
                StickDeadzoneValue.Text = _stickDeadzone.ToString("F2");
                GyroDeadzoneValue.Text = _gyroDeadzone.ToString("F2");
                GyroScaleValue.Text = _gyroScale.ToString("F2");
                LeftOrientScaleValue.Text = _leftOrientScale.ToString("F2");

                if (_currentMode == ControllerMode.SplitXboxController)
                {
                    ModeDescription.Text = "Both controllers control one Xbox controller. Left controller → left stick, right controller → right stick. Gyro active when touching A/B/trackpad.";
                    SplitModeSettings.Visibility = Visibility.Visible;
                    CalibrationSection.Visibility = Visibility.Collapsed;
                }

                System.Diagnostics.Debug.WriteLine("[LoadCalibration] Settings loaded successfully");

                if (_calibrationStep == CalibrationStep.Complete && _currentMode == ControllerMode.LightGun)
                {
                    if (InitOpenVR()) StartViGEmEmulation();
                }
                else if (_currentMode == ControllerMode.SplitXboxController)
                {
                    if (InitOpenVR()) StartViGEmEmulation();
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
        public int ControllerMode { get; set; }
        public float StickDeadzone { get; set; } = 0.1f;
        public float GyroDeadzone { get; set; } = 0.05f;
        public float GyroScale { get; set; } = 0.5f;
        public float LeftOrientScale { get; set; } = 0.5f;
    }

    private struct Point3D
    {
        public double X, Y, Z;
        public Point3D(double x, double y, double z) { X = x; Y = y; Z = z; }
    }
}
