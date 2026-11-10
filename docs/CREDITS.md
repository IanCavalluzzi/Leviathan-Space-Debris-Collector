# Credits and provenance

Leviathan was developed for UB Hacking 2026. The repository is maintained under [IanCavalluzzi](https://github.com/IanCavalluzzi).

## Mechanical inspiration

The team adapted published spiral-finger geometry and developed the surrounding assembly and actuation integration in Autodesk Fusion 360. The source research is **SpiRobs: Logarithmic Spiral-shaped Robots for Versatile Grasping Across Scales**, by Zhanchi Wang, Nikolaos M. Freris, and Xi Wei: [paper](https://arxiv.org/abs/2303.09861). The grouped array sequence follows Figure 6B. Research results in that paper are not measurements of this prototype.

## Software and services

- [Python](https://www.python.org/) and its standard library: host application and HTTPS/JSON requests.
- [OpenCV](https://opencv.org/): camera capture, GrabCut foreground extraction, image processing, and interface rendering.
- [NumPy](https://numpy.org/): image-array operations.
- [pySerial](https://pyserial.readthedocs.io/): USB serial communication.
- [Arduino UNO R4 core](https://github.com/arduino/ArduinoCore-renesas): firmware platform and timing interfaces.
- [Google Gemini API](https://ai.google.dev/gemini-api/docs): operator-requested snapshot target proposals. This project uses an existing hosted model; it does not train a debris detector.

The automatic capture state machine uses an on-delay dwell and one trigger per enable. [Beckhoff TON reference](https://infosys.beckhoff.com/content/1033/tcplclib_tc2_standard/74406539.html) · [Beckhoff rising-edge reference](https://infosys.beckhoff.com/content/1033/tcplclib_tc2_standard/74391563.html)

## AI assistance disclosure

OpenAI Codex assisted with implementation, debugging, tests, research, documentation, and repository preparation. Gemini proposes targets from operator-requested camera snapshots. The operator confirms the proposal before local tracking starts.
