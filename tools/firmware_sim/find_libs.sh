# Sourced by the run.sh scripts: finds the DaisyDuino and MIDI Library sources
# (Arduino's library folder on Linux / macOS / Windows-Git-Bash), or takes
# DAISYDUINO=... MIDILIB=... from the environment.
for d in "$HOME/Arduino/libraries" "$HOME/Documents/Arduino/libraries"; do
  [ -z "$DAISYDUINO" ] && [ -d "$d/DaisyDuino/src" ] && DAISYDUINO="$d/DaisyDuino/src"
  if [ -z "$MIDILIB" ]; then
    for m in "$d/MIDI_Library/src" "$d/MIDI Library/src"; do [ -d "$m" ] && MIDILIB="$m"; done
  fi
done
[ -n "$DAISYDUINO" ] || { echo "DaisyDuino not found: set DAISYDUINO=<path to DaisyDuino/src>"; exit 1; }
