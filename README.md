# VR Light Gun

Use your VR controller as a light gun for emulators and games. This tool tracks your Valve Index (or other SteamVR-compatible) controller and emulates an Xbox 360 controller, mapping your pointing direction to the analog stick for light gun-style aiming.

## Requirements

- Windows 10/11
- .NET 9.0 Runtime
- SteamVR
- [ViGEmBus Driver](https://github.com/nefarius/ViGEmBus/releases) (for virtual controller emulation)

## How It Works

1. The app tracks your VR controller's position and orientation via SteamVR
2. A -45° pitch offset is applied to simulate holding a gun naturally
3. You calibrate by pointing at 3 corners of your screen (Top-Left, Top-Right, Bottom-Right)
4. The app calculates where your pointing ray intersects the virtual screen plane
5. Screen coordinates are mapped to an emulated Xbox 360 left stick

## Features

- **Multiple Controller Support**: Enable left hand, right hand, or both controllers simultaneously
- **Shared Calibration**: Calibrate once with any controller, and all controllers use the same screen calibration
- **Multiple Virtual Controllers**: Each enabled VR controller creates its own virtual Xbox 360 controller
- **Works with Any SteamVR Controller**: Valve Index, HTC Vive, Oculus/Meta Quest (via Link), and other SteamVR-compatible controllers

## Usage

1. **Install ViGEmBus** - Download and install from the link above
2. **Start SteamVR** - Make sure your VR headset and controller are tracked
3. **Run VRLightGun.exe**
4. **Select Controllers** - Check the boxes for which controllers you want to use (Left Hand, Right Hand, or both)
5. **Calibrate**:
   - Click "Start Calibration"
   - Point at the **top-left** corner of your monitor and pull the trigger
   - Point at the **top-right** corner and pull the trigger
   - Point at the **bottom-right** corner and pull the trigger
6. **Play!** - The calibration is saved to `calibration.json` and loads automatically next time

## Controller Mapping

| VR Controller | Xbox 360 |
|---------------|----------|
| Trigger | A Button |
| Grip | B Button |
| Menu Button | X Button |
| Trackpad Press | Y Button |
| System Button | Start |
| Trackpad Touch | Back |
| Pointing Direction | Left Stick |

## Debug Window

A 3D debug visualization shows:
- Your controller position (green sphere)
- Calibration points (colored spheres: Yellow=TL, Cyan=TR, Magenta=BR)
- The pointing ray (orange line)
- The calibrated screen plane (white outline)

## Building

```bash
dotnet build
dotnet run
```

## License

MIT
