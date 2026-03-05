
using System.Windows;
using System.Windows.Forms;
using System.Drawing;

namespace VRLightGun;

/// <summary>
/// Interaction logic for App.xaml
/// </summary>
public partial class App : System.Windows.Application
{
    private NotifyIcon? _trayIcon;
    private MainWindow? _mainWindow;

    protected override void OnStartup(System.Windows.StartupEventArgs e)
    {
        base.OnStartup(e);

        _mainWindow = new MainWindow();
        _mainWindow.Hide();
        _mainWindow.WindowState = WindowState.Minimized;
        _mainWindow.ShowInTaskbar = false;

        _trayIcon = new NotifyIcon();
        var iconStream = GetResourceStream(new Uri("bear.ico", UriKind.Relative))?.Stream;
        _trayIcon.Icon = iconStream != null ? new Icon(iconStream) : SystemIcons.Application;
        _trayIcon.Visible = true;
        _trayIcon.Text = "VR Light Gun";

        var contextMenu = new ContextMenuStrip();
        var showItem = new ToolStripMenuItem("Show Window");
        showItem.Click += (s, ev) => { System.Diagnostics.Debug.WriteLine("[ShowWindow] Clicked"); ShowMainWindow(); };
        var debugItem = new ToolStripMenuItem("Debug View");
        debugItem.Click += (s, ev) => { System.Diagnostics.Debug.WriteLine("[DebugView] Clicked"); _mainWindow?.ToggleDebugWindow(); };
        var exitItem = new ToolStripMenuItem("Exit");
        exitItem.Click += (s, ev) => { System.Diagnostics.Debug.WriteLine("[Exit] Clicked"); ExitApp(); };
        contextMenu.Items.Add(showItem);
        contextMenu.Items.Add(debugItem);
        contextMenu.Items.Add(new ToolStripSeparator());
        contextMenu.Items.Add(exitItem);
        _trayIcon.ContextMenuStrip = contextMenu;
        
        contextMenu.Opening += (s, ev) => System.Diagnostics.Debug.WriteLine("[ContextMenu] Opening");
        contextMenu.Closing += (s, ev) => System.Diagnostics.Debug.WriteLine("[ContextMenu] Closing");

        _trayIcon.DoubleClick += (s, ev) => ShowMainWindow();
    }

    private void ShowMainWindow()
    {
        if (_mainWindow == null)
            return;
        _mainWindow.Show();
        _mainWindow.WindowState = WindowState.Normal;
        _mainWindow.Activate();
        _mainWindow.ShowInTaskbar = true;
    }

    private void ExitApp()
    {
        System.Diagnostics.Debug.WriteLine("[ExitApp] Called");
        _trayIcon?.Dispose();
        _mainWindow?.Close();
        Shutdown();
    }

    protected override void OnExit(System.Windows.ExitEventArgs e)
    {
        System.Diagnostics.Debug.WriteLine("[OnExit] Called");
        _trayIcon?.Dispose();
        base.OnExit(e);
    }
}

