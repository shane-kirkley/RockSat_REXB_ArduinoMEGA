 /*
  * TIMER SETUP - currently not used.
  */
const int timer1_TCNT = 34286; // (65536 - (16MHz/256)) / 2 -> 1 Hz
const uint8_t rearCamDelay = 100; // rearCamDelay * 500 ms = seconds to delay rear picture
uint8_t timer1_count = 0;         // counts up to rearCamDelay.
 noInterrupts();           // disable global interrupts
 TCCR1A = 0;
 TCCR1B = 0;
 TCNT1 = timer1_TCNT;   // preload timer
 TCCR1B |= (1 << CS12);    // 256 prescaler 
 TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
 interrupts();        // enable global interrupts

 // timer1: was used to delay image taking - unused now.
ISR(TIMER1_OVF_vect) {
 TCNT1 = timer1_TCNT;   // preload timer
 timer1_count++;
 if(timer1_count > rearCamDelay) {
   rearCamTrigger = true;
   timer1_count = 0;
   TIMSK1 = 0; // disable timer1 interrupts
 }
}