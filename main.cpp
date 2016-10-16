#include <WProgram.h>

int main(void) 
{
//  initWiring();
  portMode(3, OUTPUT);
  portMode(4, OUTPUT);
  sei();
  setup();
  for(;;) {
    loop();
  }
}

