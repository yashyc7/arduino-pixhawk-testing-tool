Steoss:

## ✅ Goal:

Run a **virtual Pixhawk (SITL)** that:

* Accepts MAVLink commands from your Arduino
* Sends telemetry that can be observed in a Python Tkinter GUI

---

## 🧰 Step-by-step setup (Ubuntu)

### 🟢 Step 1: Install Dependencies

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-dev python3-setuptools \
    g++ make libtool libxml2-dev libxslt1-dev libncurses-dev \
    python3-opencv openjdk-11-jdk
```

---

### 🟢 Step 2: Clone ArduPilot & Init Submodules

```bash
cd ~/Desktop  # or wherever you want
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

---

### 🟢 Step 3: Install ArduPilot dependencies

```bash
cd Tools/environment_install/
./install-prereqs-ubuntu.sh -y
. ~/.profile
```

---

### 🟢 Step 4: Build SITL (ArduCopter firmware)

```bash
cd ~/Desktop/ardupilot  # go back to root of repo
./waf configure --board sitl
./waf copter
```

✅ This builds the **ArduCopter SITL** firmware that behaves like a real Pixhawk.

---

### 🟢 Step 5: Start SITL with Arduino Output

Connect your Arduino via USB. Find the serial device:

```bash
ls /dev/ttyUSB*
```

You may see `/dev/ttyUSB0` or similar.

Then run SITL:

```bash
cd ~/Desktop/ardupilot/ArduCopter
../../Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
    --console --map --out=/dev/ttyUSB0
```

> Replace `/dev/ttyUSB0` with your actual Arduino port

source ~/venv-ardupilot/bin/activate
&& cd ~/Desktop/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map --out=127.0.0.1:14551 --out=/dev/ttyACM0,57600

sim_vehicle.py -f quadplane --console --map  --out=127.0.0.1:14551
param set ARMING_CHECK 0
wp load mission.txt
wp list           # optional - to verify waypoints
mode AUTO
arm throttle