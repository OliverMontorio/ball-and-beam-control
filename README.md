# ESP32 Ball-and-Beam Control System

A low-cost physical feedback-control project built around an **ESP32-C3**, **VL53L4CD time-of-flight sensor** and **MG996R servo**.  
The system measures the position of a ping-pong ball and adjusts the beam angle using an experimentally tuned **PD controller**, with additional filtering and friction compensation developed through physical testing.

<p align="center">
  <img src="media/final_build_photos/final_ball_beam.png" alt="Completed ESP32 ball-and-beam control system" width="100%">
</p>

## Demonstration

The controller was tested from multiple starting positions and setpoints, with the beam automatically adjusting to bring the ball towards the commanded position.

**Closed-loop position control:**  
[▶ Watch demonstration](media/demo_videos/demo_185mm_1.mp4)

**Full-length demonstration:**  
[▶ Watch demonstration](media/demo_videos/full_length_best.mp4)

> The VL53L4CD could not reliably track the ping-pong ball across the full beam. For the full-length demonstration, an initial open-loop motion brings the ball into the reliable sensing region before control is handed over to the PD controller.
