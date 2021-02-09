int state;

const int PRELAUNCH = 0;
const int LAUNCHED = 1;
const int PARTIAL_RELEASE = 2;  // after first parachute line is cut
const int FULL_RELEASE = 1;  // after second parachute line is cut
const int LANDED = 4;

void setup() {
  
}

void loop() {
  switch(state) {
    case PRELAUNCH:
      prelaunch_loop();
      break;
    case LAUNCHED:
      launched_loop();
      break;
    case PARTIAL_RELEASE:
      partial_release_loop();
      break;
    case FULL_RELEASE:
      full_release_loop();
      break;
    case LANDED:
      landed_loop();
      break;
  }
}
