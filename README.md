# 🚗 PID Line Follower Robot (Arduino)

This Arduino project is a **PID-controlled Line Follower Robot** that uses three IR sensors to follow a black line on a white surface. The robot uses a basic PID control algorithm to adjust its motor speeds in real-time for smooth and accurate line tracking.

---

## 🔧 Components Required

| Component | Quantity | Purpose | Buy Link |
|----------|----------|---------|----------|
| **Arduino Uno** | 1 | Microcontroller to read sensors, compute PID, and control motors | [Buy on Amazon](https://amzn.to/3S1P7na) |
| **L298N Motor Driver** | 1 | Drives two motors with directional and PWM control | [Buy on Amazon](https://amzn.to/4aTXsQw) |
| **Li-Po Battery (7.4V or 11.1V)** | 1 | Powers the entire system (motors and Arduino) | [Buy on Amazon](https://amzn.to/3UzEG9z) |
| **Buck Converter (Optional)** | 1 | Steps down LiPo voltage to 5V for Arduino safely | [Buy on Amazon](https://amzn.to/3KZQHtP) |
| **BO Motors + Wheels** | 2 | Provide mobility to the robot | [Buy on Amazon](https://amzn.to/3yULRCl) |
| **Caster Wheel** | 1 | Balances the robot with free rotation | [Buy on Amazon](https://amzn.to/3UnrPiY) |
| **Robot Chassis** | 1 | Physical base to mount all components | [Buy on Amazon](https://amzn.to/3V0G2ej) |
| **Mini Breadboard** | 1 | Quick wiring/testing of circuits | [Buy on Amazon](https://amzn.to/4bGgecH) |
| **PCB (optional)** | 1 | For permanent soldering & compact layout | [Buy on Amazon](https://amzn.to/4aW3P6f) |
| **IR Sensors (TCRT5000 or similar)** | 3 | Detect black line on white surface (or vice versa) | [Buy on Amazon](https://amzn.to/3KZLTy0) |
| **Male-to-Male, Male-to-Female Jumper Wires** | 20+ | Connect all modules without soldering | [Buy on Amazon](https://amzn.to/3yXQYZM) |


---

## 📸 Images & Video

Place your image in an `images/` folder in your repo and include it like this:
![Line Follower Robot](images/robot.jpg)

---

# 🧠 How Each Component Works

- **Arduino Uno:** Acts as the brain, reads IR sensor values, calculates PID, and adjusts motor speeds.

- **L298N Motor Driver:** Bridges low-power Arduino and high-current motors, enabling direction and speed control.

- **Li-Po Battery:** Supplies sufficient current for motors and Arduino. Choose 7.4V or 11.1V depending on your motor rating.

- **Buck Converter:** Converts higher battery voltage to safe 5V/9V for Arduino (if not powering via USB).

- **BO Motors + Wheels:** Convert electrical energy into motion. Used for left and right drive.

- **Caster Wheel:** Provides a third balancing point, allows smooth turning.

- **Chassis:** Holds everything together and makes the robot rigid and movable.

- **Breadboard:** For testing connections before finalizing them.

- **PCB:** Permanent version of the circuit for reliability and compactness.

- **IR Sensors:** Detect black/white contrast. Give analog or digital output depending on configuration.

- **Jumper Wires:** Connect everything without needing to solder.

---

# 🔌 Pin Configuration

| Component         | Arduino Pin |
|-------------------|-------------|
| IR Left           | A4          |
| IR Center         | A3          |
| IR Right          | A5          |
| Left Motor ENA    | 6           |
| Left Motor IN1    | 9           |
| Left Motor IN2    | 10          |
| Right Motor ENB   | 5           |
| Right Motor IN3   | 7           |
| Right Motor IN4   | 8           |

---

# ⚙️ PID Constants

```cpp
float kp = 120;
float ki = 0.0;
float kd = 70;
```

### PID Constants Explanation

- **kp** – Proportional constant. Affects how aggressively the robot reacts to errors.  
- **ki** – Integral constant. Helps eliminate accumulated small errors (often kept 0 for line following).  
- **kd** – Derivative constant. Reacts to rate of error change. Helps prevent oscillations.  

>* Note - I gave 9V power supply to battery, if you are using 12V battery then use lower value of kp and kd like (kp=70 and kd=40)

_Tune these values based on your robot’s turning behavior._

---

### 📊 How It Works

- IR sensors continuously read surface reflectivity.  
- A PID controller calculates the position error based on sensor readings.  
- Based on PID output, the motor speeds are adjusted:  
  - If the robot is veering left, speed of right motor is increased (or left decreased).  
  - If it's veering right, speed of left motor is increased.  
- This lets the robot follow the path accurately and smoothly.

---

### 💻 Code Upload Instructions

1. Connect your Arduino Uno to your PC via USB cable.  
2. Open the `.ino` file in the Arduino IDE.  
3. Go to **Tools > Board > Arduino Uno**.  
4. Go to **Tools > Port** and select the correct COM port.  
5. Click **Upload** (the ➡️ arrow icon in the IDE).  
6. (Optional) Open the Serial Monitor to see debug values from IR sensors.
