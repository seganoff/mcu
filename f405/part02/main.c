void SystemInit(void){}
void _init(void) {}
/**
 * Main program.
 */
int main(void) {
volatile int val = 0;
  // To test RAM '.data' section initialization:
  //static int dont_panic = 42;
  while (1) {
    val += 1;
  }
}
