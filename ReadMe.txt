Interface to finger sensor and glove data. Needed for current hand status to visualize it correctly.
Not needed for Tracking!


Usage : cyberGloveServer [options]:
  -h [ --help ]                                             show help screen
  --dbname arg                                              database name (on localhost) NOT IMPLEMENTED YET
  -r [ --glove-right ]                                      use right glove
  -l [ --glove-left ]                                       use left glove
  --calibration-file-left arg (=GloveCalibrationRight.cal)  left glove calibration file
  --calibration-file-right arg (=GloveCalibrationRight.cal) right glove calibration file
  --tty-right arg (=/dev/ttyUSB0)                           right glove tty (e.g. /dev/ttyS0
  --tty-left arg (=/dev/ttyUSB1)                            left glove tty (e.g. /dev/ttyS1
  --register-as-right arg (=GloveRight)                     right glove Name
  --register-as-left arg (=GloveLeft)                       left glove Name
  -d [ --debug-level ] arg (=0)                             the more, the higher ;)



-----------------------------------------------------------------------

To Start the Programm:

./bin/gloveServer_node -r -l --calibration-file-right /home/staff/jaekel/diplom/PbD/CyberGlove/calibrationData/RainerRight210111.cal --calibration-file-left /home/staff/jaekel/diplom/PbD/CyberGlove/calibrationData/RainerLeft080910.cal --tty-right /dev/ttyD0 --tty-left /dev/ttyD1 -d 0
