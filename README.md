# TesLorean-Instrument-Controller
Arduino code driving the TesLorean instrument cluster gauges, lights, and OLED

This project contains the code necessary to control the instrument cluster in the TesLorean.  The instrument cluster is a complete replacement for the stock DeLorean instruments.  It uses GM gauge stepper motors to control the gauges indicating speed, range, power, temps, etc.  The cluster includes a number of warning lights (arranged similar to stock) but with very different purposes.  There are also two small OLED displays under the main gauges.  These will display status information, warning messages (if present), and any trouble codes.

The instrument controller will listen for status information from the other control modules in the TesLorean and update the gauge information, lights, and OLED displays accordingly.
