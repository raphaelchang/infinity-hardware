#include <kinetis.h>

unsigned short adc0val;
unsigned short adc1val;

void setup_adc()
{
  /* MODE = 0x3 - single-ended 16-bit conversion */
  ADC0_CFG1 |= ADC_CFG1_MODE(0x3);
  /* ADHSC = 1 - very high speed operation */
  ADC0_CFG2 |= ADC_CFG2_ADHSC ;
  /* ADTRG = 1 - hardware trigger selected */
  ADC0_SC2 |= ADC_SC2_ADTRG ;
  /* AIEN = 1 - conversion complete interrupt enabled, ADCH = 0x9 - Channel 9 is selected as
  SE input */
  ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(0x9); /* corresponding ADC0_RA */
  /* enable clock for peripherals */
  SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC
   | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE ;
  SIM_SCGC6 |= SIM_SCGC6_FTM0 | SIM_SCGC6_PDB | SIM_SCGC6_ADC0;
}

unsigned short ADC0_IRQHandler(void)
{ 
  ADC0_SC1A = 0;
  /* toggle PTC18 to see when ADC interrupt is called */
 //GPIOC_PTOR = 1UL << 18;
 /* Save the results and clear the COCO flag */
 while(true)
 if ( ADC0_SC1A & ADC_SC1_COCO )
 {
 adc0val = (unsigned short) ADC0_RA;
 return adc0val;
 }
}
