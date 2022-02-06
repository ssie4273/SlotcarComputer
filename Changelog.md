# Changelog
## Version 2.1.0
- green LED left or right on starting light indicating leading car.

## Version 2.0.0
- btn1 and btn4 switch between laptime and scaled velocity.
- Laptimes below float minLapTime will be discarded. issue #16


## Version 1.3.0
- Current raceduration shown on LCD
- longer IR_off_cycle

## Version 1.2.0
- different threshold for track A. Shortened LED cycle isn starting lights.
- Eye candy for LED blinking red at end of race.
- end result is blanked for second driver if only one driver is racing against the clock.

## Version 1.1.0
- end result shown as overview

## Version 1.0.1
- fix in runde_A  (but why??) readz for tag 1.0.1

## Version 1.0.0
- Merge branch 'main' of github.com:ssie4273/SlotcarComputer into main
- End race conditions. ready for tag v1.0.0
- Taste 4: press to start over. Debugging...

## Version 0.3
- fixing the fix for issue #6
- fix for issue #6 in case of false start
- fix for issue #6
- end condition for time and lap count race
- check for end-race condition
- LED running lights: improved loop

## Version 0.2
- Code Cleanup
- Results diplayed on I2C LCD.

## Version 0.1
- small fix in Serial printout
- remove of unnecessary reasoutLanes().
- infinite raceLoop() with output on Serial.print IR value is averaged over ten readings. ready for first pre-releae version!
- MERGE: Smoothing merged from testtools
- Smoothing of IR bridge values.
- mapped analogreads
- readoutLanes() in setup(): resetting diffMax_A to 0
- using global variable for IR diff
- display IR diffs
- different thresholds for lane A and B
- cleanup Serial.print logging
- react on pressign button (not releasing)
- Debouncing pushbuttons with library Bounce2: Fixing issues:  not able to go back page 1 on settings: new #4  Pushbuttons not debounced #1
- define settings when switch si set to "settings" set_race (Pin 7)
- settings as separate function
- Settings can be edited. NEED FIX: debounce of pushbuttons does not work very well. Currently workaround with simple delay(50)
- Initial commit
