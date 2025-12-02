# Physical Dice

Physical Dice is a realistic physics-based dice simulation tool that uses 3D rigid body dynamics to roll virtual polyhedral dice. Unlike simple random number generators, this tool simulates the actual physics of dice rolling, including gravity, collision detection, friction, and angular momentum.

## Features

- Realistic 3D physics simulation using rigid body dynamics
- Support for standard dice (D4, D6, D8, D12, D20) and custom N-sided dice (4-254 sides)
- Multiple output formats (text, JSON, CSV)
- Batch rolling capabilities
- Configurable physics parameters
- Convex polyhedral dice with proper mass and inertia calculations based on Mirtich (1996)

## Usage

Example usage:
```
roll 2D6 1D20
```

Roll multiple dice expressions simultaneously. The format is `{count}D{sides}`, such as:
- `2D6` - Two 6-sided dice
- `1D20` - One 20-sided die
- `3D4` - Three 4-sided dice

### Options

- `-t, --time <time>` - Maximum simulation time in seconds (default: 8.0)
- `-o, --output <output>` - Output format: text, json, csv (default: text)
- `-v, --verbose` - Verbose output
- `--mass <mass>` - Mass for each die (default: 0.17)
- `--batch <batch>` - Number of rolls for batch mode (default: 1)

### Examples

```
# Roll 2 six-sided dice
roll 2D6

# Roll 1 twenty-sided die with JSON output
roll 1D20 --output json

# Roll multiple dice types
roll 2D6 1D8 1D20

# Batch roll 5 times
roll 1D6 --batch 5

# Custom simulation time
roll 1D20 --time 10.0
```

## Install

The latest pre-built binary can be found at [here](https://github.com/DaZuo0122/PhysicalDice/releases/latest/).


For Debian/Ubuntu user, it can be installed by:

```bash
wget -q https://github.com/DaZuo0122/PhysicalDice/releases/latest/download/roll_x86_64.deb -O roll_x86_64.deb && sudo apt install -f ./roll_x86_64.deb
```

## Building from Source

The project consists of two components:
- `roll`: The main CLI application
- `libpdice`: The physics simulation library

To build:
```
cargo build --release
```

## How It Works

The simulation uses a realistic physics model where dice are represented as convex polyhedra with accurate mass distribution calculated using the method described by Brian Mirtich (1996). Each die is assigned random initial position and velocity, and then simulated using semi-implicit integration with collision detection against a ground plane.

The physics simulation includes:
- Gravity
- Collision detection and response
- Friction (static and dynamic)
- Rolling resistance
- Sleep detection (when dice come to rest)

Physics parameters are tuned to match real-world dice rolling behavior, ensuring that the final rolled value depends on actual physical simulation rather than simple randomization.
