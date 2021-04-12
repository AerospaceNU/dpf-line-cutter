# Line Cutter State Machine

The line cutter uses a simple progression of states. With the exception of the first two states (`waiting` and `armed`), which can be visited multiple times, the states advance in a linear manner.

| State | Requirement | Next state |
|-------| ----------------------- | ---------- |
| `waiting` | Receive `!arm` signal from BLE central. | `armed ` |
| `armed` | Receive `!disarm` signal from BLE central. <br> Photoresistor detects bright light.| `waiting` <br> `deployed` |
| `deployed` | Altitude goes below first disreef height. | `partial disreef` |
| `partial disreef` | Altitude goes below second disreef height. | `full disreef` |
| `full disreef` | Speed is below landing limit speed. | `landed` |
| `landed` | N/A | N/A |

## Transitions

All state transitions add data to a state transition log on the NRF52 chip. In addition, some transitions execute special actions first.

* `armed` -> `deployed`: Existing state transition log is deleted.
* `deployed` -> `partial disreef`: PWM begins on first nichrome pin.
* `partial disreef` -> `full disreef`: PWM begins on second nichrome pin.

The PWM on each nichrome pin is started during a state transition, and the time of this transition is marked. Once the defined PWM duration has passed since the transition, the PWM will end.
