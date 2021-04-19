# Line cutter schematics

This folder is organized as follows:
  * `batt-and-reg.cam`: CAM file used for JLC
  * `lineCutterR<N>-.zip`: Production files sent to fab
  * `LineCutter`: eagle project containing all the actual files
    * `Line Cutter COTS.lbr`: common parts used (flash memory, nRF daughterboards, etc)
    * `lineCutterR1`: first PCB ordered (nRF52832, December 2020)
    * `lineCutterR2`: ESP32-S2 based, ordered spring 2021 (1 built, no software written for it)
    * `lineCutterR3`: nRF52840-based, ordered spring 2021. Flew Amesbury 3/20.