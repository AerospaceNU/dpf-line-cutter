int state;

const int PRELAUNCH = 0;
const int LAUNCHED = 1;
const int DEPLOYED = 2;
const int PARTIAL_DISREEF = 3;  // after first parachute line is cut
const int FULL_DISREEF = 4;  // after second parachute line is cut
const int LANDED = 5;

void setup() {
  
}

void loop() {
  switch(state) {
    case PRELAUNCH:
      prelaunch();
      break;
    case LAUNCHED:
      launched();
      break;
    case DEPLOYED:
      deployed();
      break;
    case PARTIAL_DISREEF:
      partial_disreef();
      break;
    case FULL_DISREEF:
      full_disreef();
      break;
    case LANDED:
      landed();
      break;
  }
}

/*************************
 *      MAIN STATES      *
 ************************/

void prelaunch() {

}

void launched() {

}

void deployed(){
  
}

void partial_disreef() {
  
}

void full_disreef() {
  
}

void landed() {
  
}

/*************************
 *        HELPERS        *
 ************************/

 
