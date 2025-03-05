# 2025-Reefscape
2025 Season - Reefscape

# RoboRIO Hardware I/O Configuration
## Digital I/O
| Port  | Device                 |
|:-----:|------------------------|
| 0     | Coral Limit Switch     |
| 1     | Bore Encoder Channel A |
| 2     | Bore Encoder Channel B |
| 3     | Elevator Limit Switch  |
| 4     |                        |
| 5     |                        |
| 6     |                        |
| 7     |                        |
| 8     |                        |
| 9     |                        |

## PWM
| Port  | Device                 |
|:-----:|------------------------|
| 0     |                        |
| 1     |                        |
| 2     |                        |
| 3     |                        |
| 4     |                        |
| 5     |                        |
| 6     |                        |
| 7     |                        |
| 8     | LED Lights Left        |
| 9     | LED Lights Right       |

## Relay
| Port  | Device                 |
|:-----:|------------------------|
| 0     |                        |
| 1     |                        |
| 2     |                        |
| 3     |                        |


## Analog In
| Port  | Device                 |
|:-----:|------------------------|
| 0     |                        |
| 1     |                        |
| 2     |                        |
| 3     |                        |


# CAN ID Assignments
## Drivebase CAN Devices (IDs 1-12)
| Module           | Drive Motor ID | Angle Motor ID | Encoder ID |
|------------------|:--------------:|:--------------:|:----------:|
| Front Left (FL)  | 1              | 2              | 3          |
| Front Right (FR) | 4              | 5              | 6          |
| Back Left (BL)   | 7              | 8              | 9          |
| Back Right (BR)  | 10             | 11             | 12         |

## Coral CAN Devices (IDs 20-29)
| Device           | CAN ID   |
|------------------|:--------:|
| Intake & Outake  | 20       |
|                  | 21       |
|                  | 22       |
|                  | 23       |
|                  | 24       |
|                  | 25       |
|                  | 26       |
|                  | 27       |
|                  | 28       |
|                  | 29       |

## Elevator CAN Devices (IDs 30-39)
| Device           | CAN ID   |
|------------------|:--------:|
| Right Motor      | 30       |
| Left Motor       | 31       |
|                  | 32       |
|                  | 33       |
|                  | 34       |
|                  | 35       |
|                  | 36       |
|                  | 37       |
|                  | 38       |
|                  | 39       |
