
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
        _trayIcon.Icon = SystemIcons.Application;
        _trayIcon.Visible = true;
        _trayIcon.Text = "VR Light Gun";

        var contextMenu = new ContextMenuStrip();
        var showItem = new ToolStripMenuItem("Show Window");
        showItem.Click += (s, ev) => ShowMainWindow();
        var debugItem = new ToolStripMenuItem("Debug View");
        debugItem.Click += (s, ev) => _mainWindow?.ToggleDebugWindow();
        var exitItem = new ToolStripMenuItem("Exit");
        exitItem.Click += (s, ev) => ExitApp();
        contextMenu.Items.Add(showItem);
        contextMenu.Items.Add(debugItem);
        contextMenu.Items.Add(new ToolStripSeparator());
        contextMenu.Items.Add(exitItem);
        _trayIcon.ContextMenuStrip = contextMenu;

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
        _trayIcon?.Dispose();
        _mainWindow?.Close();
        Shutdown();
    }

    protected override void OnExit(System.Windows.ExitEventArgs e)
    {
        _trayIcon?.Dispose();
        base.OnExit(e);
    }
}

