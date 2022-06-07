### GitHub Repository
https://github.com/osu-usli/Payload

# OSU USLI 2021-2022 Payload software

This repository contains the software for the Arduino on the rocket payload. This Arduino is connected to an AltIMU-v10 inertial measurement unit, a SX1280 radio time-of-flight and communication module, and the two Raspberry Pis used for vision processing.

The Arduino is in charge of detecting launch and landing, controlling time-of-flight measurements & communications with the base station, and estimating the rocket's final landing site. The basic program flow is:

- Initialize the hardware. Set the lines to the Raspberry Pis to low to indicate that the vision processing software in the Pis should stand by waiting for launch.
- Until a landing is detected:
    - Send time-of-flight requests continuously, always keeping track of the most recently measured distance.
    - Measure acceleration using the IMUs, running the output through a low-pass filter.
        - If the acceleration exceeds 4 Gs and the "launched" flag has not yet been set, set the launched flag & set the lines to the Raspberry Pis to high, indicating that the vision processing software should begin capturing images.
        - If the "launched" flag has been set and the acceleration has stayed between 0.8 and 1.2 Gs for the past 10 seconds, we've detected a landing.
    - If the "launched" flag is set and the time since launch exceeds 150 seconds, we've detected a landing. (150 seconds is about 30 seconds beyond the theoretical maximum amount of time the rocket could possibly spend in the air. This timeout exists so that we detect a landing even if the rocket never really comes to rest due to windy conditions)
- Once we detect a landing, set the lines to the Raspberry Pis to low to indicate that the vision processing software should stop collecting images and start analyzing the results.
- Wait for the Raspberry Pis to return vision processing results over serial.
- Use the vision processing results to estimate the rocket's bearing to the tent. Combine this with the latest time-of-flight distance measurement to compute a grid square.
- Repeatedly transmit the grid square to the base station using the time-of-flight module.

Besides the standard Arduino SDK, this software depends on the [LSM6](https://www.arduino.cc/reference/en/libraries/lsm6/) and [LPS](https://www.arduino.cc/reference/en/libraries/lps/) libraries, which you can install using the Library Manager in the Arduino IDE. Once the libraries are installed, the `Payload.ino` file will compile and run using the Arduino IDE.
