# Building CARLA 0.9.15 from Source on Ubuntu 22.04

This guide documents the working dependency and build procedure for **CARLA 0.9.15** on **Ubuntu 22.04 (Jammy)** using **system Python 3.10** and **Unreal Engine 4.26 (carla branch)**.

Dependency fixes are based on:
https://github.com/carla-simulator/carla/issues/6381#issuecomment-1560837119

---

# 1. System Dependencies

Install required packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  clang-12 lld-12 \
  cmake ninja-build \
  libvulkan1 \
  libpng-dev libtiff5-dev libjpeg-dev \
  tzdata sed curl unzip autoconf libtool rsync \
  libxml2-dev git aria2 libstdc++-12-dev libbz2-dev
```

Set Clang 12 as default:

```bash
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-12/bin/clang++ 180
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-12/bin/clang 180
```

Install Python helper:

```bash
pip3 install distro
```

This guide assumes **system Python 3.10**.

---

# 2. Build Unreal Engine 4.26 (CARLA Branch)

Clone Unreal:

```bash
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
cd ~/UnrealEngine_4.26
```
Note: you github account must be linked to an Epic games account for access to the UE4 repo. Follow this guide: https://www.epicgames.com/help/en-US/account-c-202300000001645/linked-accounts-c-202300000001754/how-do-i-link-my-unreal-engine-account-with-my-github-account-a202300000012521

Build Unreal:

```bash
./Setup.sh
./GenerateProjectFiles.sh
make
```

Launch UE4 Editor to confirm build:

```bash
cd ~/UnrealEngine_4.26/Engine/Binaries/Linux
./UE4Editor
```

The Unreal Editor should launch successfully.

Set `UE4_ROOT` environment variable:

```bash
export UE4_ROOT=~/UnrealEngine_4.26
```

(Optional: add to `~/.bashrc`)

---

# 3. Clone and Checkout CARLA 0.9.15

```bash
git clone https://github.com/carla-simulator/carla
cd carla
git checkout 0.9.15
```

---

# 4. Patch `Setup.sh` for Ubuntu 22.04

Modify the following lines in `Setup.sh`.

### 4.1 Fix Boost Download (around line ~94)

Change to:

```bash
wget "https://archives.boost.io/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz" || true
```

---

### 4.2 Fix libpng Download (around line ~342)

Change to:

```bash
LIBPNG_REPO=https://github.com/glennrp/libpng/archive/refs/tags/v1.6.37.tar.gz
```

---

### 4.3 Fix libpng Extraction (around line ~356–357)

Change extraction commands to:

```bash
mv v${LIBPNG_VERSION}.tar.gz libpng-${LIBPNG_VERSION}.tar.gz
tar -xf libpng-${LIBPNG_VERSION}.tar.gz
```

---

# 5. Fix Python / setuptools Compatibility

Install required Python dependencies:

```bash
python3.10 -m pip install --user -U \
  jaraco.functools jaraco.collections jaraco.text
```

Remove incompatible setuptools versions:

```bash
rm -rf ~/.local/lib/python3.10/site-packages/setuptools*
rm -rf ~/.local/lib/python3.10/site-packages/_distutils_hack*
rm -rf ~/.local/lib/python3.10/site-packages/distutils-precedence.pth
rm -rf ~/.local/lib/python3.10/site-packages/pkg_resources*
```

Install compatible versions:

```bash
python3.10 -m pip install --user --upgrade --force-reinstall \
  "pip<24.1" \
  "setuptools==65.5.1" \
  wheel
```

---

# 6. Build Python API

```bash
make PythonAPI
```

After build completes, install the generated wheel:

```bash
cd dist/Carla_Shipping_0.9.15-dirty/LinuxNoEditor/Python/API/carla/dist
python3.10 -m pip install --user -U ./carla-0.9.15-cp310-cp310-linux_x86_64.whl
```

---

# 7. Verify CARLA Python Module

Check import:

```bash
python3.10 -c "import carla, inspect; print('carla module:', carla.__file__)"
```

Further validation:

```bash
python3.10 -c "import carla; print('ok import'); print([a for a in dir(carla) if a.startswith('Client') or a.startswith('World') or a.startswith('Weather')][:20])"
```

---

# 8. Fix `python` vs `python3` Issue

Option A — Install shim:

```bash
sudo apt-get install -y python-is-python3
```

Option B — Patch build script:

```bash
sed -i 's/\bpython\b/python3/g' ~/PathToCarlaInstall/carla/Util/BuildTools/BuildCarlaUE4.sh
```

---

# 9. Launch CARLA Editor

```bash
make launch
```

Unreal Editor should open with the CARLA project.

You can reopen the CARLA project later by opening the carla .uproject file with the UnrealEditor:

```
carla/Unreal/CarlaUE4/CarlaUE4.uproject
```

---

# 10. Import Custom Map

Follow official documentation:
https://carla.readthedocs.io/en/0.9.15/tuto_M_add_map_source/

Place `.xodr` and `.fbx` files into:

```
carla/import
```

Run:

```bash
make import
```

---

# 11. Generate Traffic Lights from `traffic_light_table.csv`

Open map:

File → Open Level  
Content/[map_package_name]/Maps/[map_name]

Set an environment variable called SUMO_TLS_TABLE_PATH to the full path location of your traffic light table in the UnrealEditor console:
Change the input type of the Output Log to 'python', then enter:
```bash
import os
os.environ["SUMO_TLS_TABLE_PATH"] = "/home/[user_name]/path/to/traffic_light_table.csv"
print(os.environ["SUMO_TLS_TABLE_PATH"])
```

Run script:

File → Execute Python Script...

Select:

FIXS/tests/Applications/SUMO_CARLA_EcoDriving/helper_scripts/unreal_placing_tls.py

---

# 12. Run CARLA World from Unreal

Click dropdown next to **Play** button  
Select:
Standalone Game